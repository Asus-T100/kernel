/*
 * EHCI HCD (Host Controller Driver) PCI Bus Glue.
 *
 * Copyright (c) 2000-2004 by David Brownell
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef CONFIG_PCI
#error "This file is PCI bus glue.  CONFIG_PCI must be defined."
#endif

/* defined here to avoid adding to pci_ids.h for single instance use */
#define PCI_DEVICE_ID_INTEL_CE4100_USB	0x2e70

/*-------------------------------------------------------------------------*/

/* called after powerup, by probe or system-pm "wakeup" */
static int ehci_pci_reinit(struct ehci_hcd *ehci, struct pci_dev *pdev)
{
	int			retval;

	/* we expect static quirk code to handle the "extended capabilities"
	 * (currently just BIOS handoff) allowed starting with EHCI 0.96
	 */

	/* PCI Memory-Write-Invalidate cycle support is optional (uncommon) */
	retval = pci_set_mwi(pdev);
	if (!retval)
		ehci_dbg(ehci, "MWI active\n");

	return 0;
}

/* enable SRAM if sram detected */
static void sram_init(struct usb_hcd *hcd)
{
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	struct pci_dev		*pdev = to_pci_dev(hcd->self.controller);
	void __iomem		*base = NULL;
	void __iomem		*addr = NULL;

	if (!hcd->has_sram)
		return;
	ehci->sram_addr = pci_resource_start(pdev, 1);
	ehci->sram_size = pci_resource_len(pdev, 1);
	ehci_info(ehci, "Found HCD SRAM at %x size:%x\n",
		ehci->sram_addr, ehci->sram_size);

	if (pci_request_region(pdev, 1, kobject_name(&pdev->dev.kobj))) {
		ehci_warn(ehci, "SRAM request failed\n");
		hcd->has_sram = 0;
		return;
	} else if (!dma_declare_coherent_memory(&pdev->dev, ehci->sram_addr,
			ehci->sram_addr, ehci->sram_size, DMA_MEMORY_MAP)) {
		ehci_warn(ehci, "SRAM DMA declare failed\n");
		pci_release_region(pdev, 1);
		hcd->has_sram = 0;
		return;
	}

	/* initialize SRAM to 0 to avoid ECC errors during entry into D0 */
	base = ioremap_nocache(ehci->sram_addr, ehci->sram_size);
	if (base == NULL) {
		ehci_warn(ehci, "SRAM init: ioremap failed\n");
		return;
	}

	addr = base;

	while (addr < base + ehci->sram_size) {
		writel(0x0, addr);
		addr = addr + 4;
	}

	iounmap(base);
}

static void sram_deinit(struct usb_hcd *hcd)
{
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	struct pci_dev *pdev = to_pci_dev(hcd->self.controller);

	if (!hcd->has_sram)
		return;
	dma_release_declared_memory(&pdev->dev);
	pci_release_region(pdev, 1);

	/* If host is suspended, SRAM backup memory should be freed */
	if (ehci->sram_swap) {
		vfree(ehci->sram_swap);
		ehci->sram_swap = NULL;
	}

}

static int sram_backup(struct usb_hcd *hcd)
{
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	void __iomem		*base;
	int			offset;

	ehci->sram_swap = vmalloc(ehci->sram_size);
	if (!ehci->sram_swap) {
		ehci_warn(ehci, "SRAM backup memory request failed\n");
		return -ENOMEM;
	}

	base = ioremap_nocache(ehci->sram_addr, ehci->sram_size);
	if (!base) {
		ehci_warn(ehci, "SRAM backeup ioremap fails\n");
		vfree(ehci->sram_swap);
		ehci->sram_swap = NULL;
		return -EFAULT;
	}

	for (offset = 0; offset < ehci->sram_size; offset += 4)
		*(u32 *)(ehci->sram_swap + offset) = readl(base + offset);

	iounmap(base);

	return 0;
}

static int sram_restore(struct usb_hcd *hcd)
{
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	void __iomem		*base;
	int			offset;

	if (!ehci->sram_swap)
		return -EFAULT;

	base = ioremap_nocache(ehci->sram_addr, ehci->sram_size);
	if (!base) {
		ehci_warn(ehci, "SRAM_restore ioremap fails\n");
		return -EFAULT;
	}

	for (offset = 0; offset < ehci->sram_size; offset += 4)
		writel(*(u32 *)(ehci->sram_swap + offset), base + offset);

	iounmap(base);
	vfree(ehci->sram_swap);
	ehci->sram_swap = NULL;

	return 0;
}


/* called during probe() after chip reset completes */
static int ehci_pci_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	struct pci_dev		*pdev = to_pci_dev(hcd->self.controller);
	struct pci_dev		*p_smbus;
	u8			rev;
	u32			temp;
	int			retval;
	int			force_otg_hc_mode = 0;

	switch (pdev->vendor) {
	case PCI_VENDOR_ID_TOSHIBA_2:
		/* celleb's companion chip */
		if (pdev->device == 0x01b5) {
#ifdef CONFIG_USB_EHCI_BIG_ENDIAN_MMIO
			ehci->big_endian_mmio = 1;
#else
			ehci_warn(ehci,
				  "unsupported big endian Toshiba quirk\n");
#endif
		}
		break;
	case PCI_VENDOR_ID_INTEL:
		if (pdev->device == 0x0811 || pdev->device == 0x0829 ||
				pdev->device == 0xE006) {
			ehci_info(ehci, "Detected Intel MID OTG HC\n");
			hcd->has_tt = 1;
			ehci->has_hostpc = 1;
#ifdef CONFIG_USB_OTG
			ehci->has_otg = 1;
#endif
			force_otg_hc_mode = 1;

#ifdef CONFIG_BOARD_REDRIDGE
			hcd->power_budget = 500;
#else
			/* For Penwell, Power budget limit is 200mA,
			 * For Cloverview, Power budget limit is 500mA */
			if (pdev->device == 0x0829)
				hcd->power_budget = 200;
			else if (pdev->device == 0xE006)
				hcd->power_budget = 500;
#endif

			hcd->has_sram = 1;
			/*
			 * Disable SRAM for CLVP A0 due to the silicon issue.
			 */
			if (pdev->device == 0xE006 && pdev->revision < 0xC) {
				ehci_info(ehci, "Disable SRAM for CLVP A0\n");
				hcd->has_sram = 0;
			}

			hcd->sram_no_payload = 1;
			sram_init(hcd);
		} else if (pdev->device == 0x0806) {
			ehci_info(ehci, "Detected Langwell MPH\n");
			hcd->has_tt = 1;
			ehci->has_hostpc = 1;
			hcd->has_sram = 1;
			hcd->sram_no_payload = 1;
			sram_init(hcd);
		} else if (pdev->device == 0x0829) {
			ehci_info(ehci, "Detected Penwell OTG HC\n");
			hcd->has_tt = 1;
			ehci->has_hostpc = 1;
		} else if (pdev->device == 0x08F2) {
			/* Check SPH enabled or not */
			if (!sph_enabled()) {
				ehci_info(ehci, "USB SPH is disabled\n");
#ifdef CONFIG_HSI_NO_MODEM
				/* All need to bypass tll mode  */
				temp = ehci_readl(ehci, hcd->regs + CLV_SPHCFG);
				temp &= ~CLV_SPHCFG_ULPI1TYPE;
				/* ULPI 1 ref-clock switch off */
				temp |= CLV_SPHCFG_REFCKDIS;
				ehci_writel(ehci, temp, hcd->regs + CLV_SPHCFG);
#endif
				return -ENODEV;
			}

			ehci_info(ehci, "Detected SPH HC\n");
			hcd->has_tt = 1;
			ehci->has_hostpc = 1;

			/* All need to bypass tll mode  */
			temp = ehci_readl(ehci, hcd->regs + CLV_SPHCFG);
			temp &= ~CLV_SPHCFG_ULPI1TYPE;
			ehci_writel(ehci, temp, hcd->regs + CLV_SPHCFG);

			temp = ehci_readl(ehci, hcd->regs + CLV_SPH_HOSTPC);
			temp |= CLV_SPH_HOSTPC_PTS;
			ehci_writel(ehci, temp, hcd->regs + CLV_SPH_HOSTPC);

			device_set_wakeup_enable(&pdev->dev, true);

			/* Set Runtime-PM flags for SPH */
			hcd->rpm_control = 1;
			hcd->rpm_resume = 0;
			pm_runtime_set_active(&pdev->dev);
		} else if (pdev->device == 0x119C) {
			ehci_info(ehci, "Detected HSIC HC 0x119D\n");
			hcd->has_tt = 1;
			ehci->has_hostpc = 1;
		} else if (pdev->device == 0x119D) {
			ehci_info(ehci, "Detected HSIC HC 0x119D\n");
			hcd->has_tt = 1;
			ehci->has_hostpc = 1;
		}
	}

	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs +
		HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase));

	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

        /* ehci_init() causes memory for DMA transfers to be
         * allocated.  Thus, any vendor-specific workarounds based on
         * limiting the type of memory used for DMA transfers must
         * happen before ehci_init() is called. */
	switch (pdev->vendor) {
	case PCI_VENDOR_ID_NVIDIA:
		/* NVidia reports that certain chips don't handle
		 * QH, ITD, or SITD addresses above 2GB.  (But TD,
		 * data buffer, and periodic schedule are normal.)
		 */
		switch (pdev->device) {
		case 0x003c:	/* MCP04 */
		case 0x005b:	/* CK804 */
		case 0x00d8:	/* CK8 */
		case 0x00e8:	/* CK8S */
			if (pci_set_consistent_dma_mask(pdev,
						DMA_BIT_MASK(31)) < 0)
				ehci_warn(ehci, "can't enable NVidia "
					"workaround for >2GB RAM\n");
			break;
		}
		break;
	}

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);
	if (force_otg_hc_mode)
		ehci_reset(ehci);

	retval = ehci_halt(ehci);
	if (retval)
		return retval;

	if ((pdev->vendor == PCI_VENDOR_ID_AMD && pdev->device == 0x7808) ||
	    (pdev->vendor == PCI_VENDOR_ID_ATI && pdev->device == 0x4396)) {
		/* EHCI controller on AMD SB700/SB800/Hudson-2/3 platforms may
		 * read/write memory space which does not belong to it when
		 * there is NULL pointer with T-bit set to 1 in the frame list
		 * table. To avoid the issue, the frame list link pointer
		 * should always contain a valid pointer to a inactive qh.
		 */
		ehci->use_dummy_qh = 1;
		ehci_info(ehci, "applying AMD SB700/SB800/Hudson-2/3 EHCI "
				"dummy qh workaround\n");
	}

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

	switch (pdev->vendor) {
	case PCI_VENDOR_ID_NEC:
		ehci->need_io_watchdog = 0;
		break;
	case PCI_VENDOR_ID_INTEL:
		ehci->need_io_watchdog = 0;
		ehci->fs_i_thresh = 1;
		if (pdev->device == 0x27cc) {
			ehci->broken_periodic = 1;
			ehci_info(ehci, "using broken periodic workaround\n");
		}
		if (pdev->device == 0x0806 || pdev->device == 0x0811
			|| pdev->device == 0x0829 || pdev->device == 0xE006) {
			ehci_info(ehci, "disable lpm for langwell/penwell\n");
			ehci->has_lpm = 0;
		}
		if (pdev->device == PCI_DEVICE_ID_INTEL_CE4100_USB) {
			hcd->has_tt = 1;
			tdi_reset(ehci);
		}
		if (pdev->subsystem_vendor == PCI_VENDOR_ID_ASUSTEK) {
			/* EHCI #1 or #2 on 6 Series/C200 Series chipset */
			if (pdev->device == 0x1c26 || pdev->device == 0x1c2d) {
				ehci_info(ehci, "broken D3 during system sleep on ASUS\n");
				hcd->broken_pci_sleep = 1;
				device_set_wakeup_capable(&pdev->dev, false);
			}
		}
		break;
	case PCI_VENDOR_ID_TDI:
		if (pdev->device == PCI_DEVICE_ID_TDI_EHCI) {
			hcd->has_tt = 1;
			tdi_reset(ehci);
		}
		break;
	case PCI_VENDOR_ID_AMD:
		/* AMD PLL quirk */
		if (usb_amd_find_chipset_info())
			ehci->amd_pll_fix = 1;
		/* AMD8111 EHCI doesn't work, according to AMD errata */
		if (pdev->device == 0x7463) {
			ehci_info(ehci, "ignoring AMD8111 (errata)\n");
			retval = -EIO;
			goto done;
		}
		break;
	case PCI_VENDOR_ID_NVIDIA:
		switch (pdev->device) {
		/* Some NForce2 chips have problems with selective suspend;
		 * fixed in newer silicon.
		 */
		case 0x0068:
			if (pdev->revision < 0xa4)
				ehci->no_selective_suspend = 1;
			break;

		/* MCP89 chips on the MacBookAir3,1 give EPROTO when
		 * fetching device descriptors unless LPM is disabled.
		 * There are also intermittent problems enumerating
		 * devices with PPCD enabled.
		 */
		case 0x0d9d:
			ehci_info(ehci, "disable lpm/ppcd for nvidia mcp89");
			ehci->has_lpm = 0;
			ehci->has_ppcd = 0;
			ehci->command &= ~CMD_PPCEE;
			break;
		}
		break;
	case PCI_VENDOR_ID_VIA:
		if (pdev->device == 0x3104 && (pdev->revision & 0xf0) == 0x60) {
			u8 tmp;

			/* The VT6212 defaults to a 1 usec EHCI sleep time which
			 * hogs the PCI bus *badly*. Setting bit 5 of 0x4B makes
			 * that sleep time use the conventional 10 usec.
			 */
			pci_read_config_byte(pdev, 0x4b, &tmp);
			if (tmp & 0x20)
				break;
			pci_write_config_byte(pdev, 0x4b, tmp | 0x20);
		}
		break;
	case PCI_VENDOR_ID_ATI:
		/* AMD PLL quirk */
		if (usb_amd_find_chipset_info())
			ehci->amd_pll_fix = 1;
		/* SB600 and old version of SB700 have a bug in EHCI controller,
		 * which causes usb devices lose response in some cases.
		 */
		if ((pdev->device == 0x4386) || (pdev->device == 0x4396)) {
			p_smbus = pci_get_device(PCI_VENDOR_ID_ATI,
						 PCI_DEVICE_ID_ATI_SBX00_SMBUS,
						 NULL);
			if (!p_smbus)
				break;
			rev = p_smbus->revision;
			if ((pdev->device == 0x4386) || (rev == 0x3a)
			    || (rev == 0x3b)) {
				u8 tmp;
				ehci_info(ehci, "applying AMD SB600/SB700 USB "
					"freeze workaround\n");
				pci_read_config_byte(pdev, 0x53, &tmp);
				pci_write_config_byte(pdev, 0x53, tmp | (1<<3));
			}
			pci_dev_put(p_smbus);
		}
		break;
	case PCI_VENDOR_ID_NETMOS:
		/* MosChip frame-index-register bug */
		ehci_info(ehci, "applying MosChip frame-index workaround\n");
		ehci->frame_index_bug = 1;
		break;
	}

	/* optional debug port, normally in the first BAR */
	temp = pci_find_capability(pdev, 0x0a);
	if (temp) {
		pci_read_config_dword(pdev, temp, &temp);
		temp >>= 16;
		if ((temp & (3 << 13)) == (1 << 13)) {
			temp &= 0x1fff;
			ehci->debug = ehci_to_hcd(ehci)->regs + temp;
			temp = ehci_readl(ehci, &ehci->debug->control);
			ehci_info(ehci, "debug port %d%s\n",
				HCS_DEBUG_PORT(ehci->hcs_params),
				(temp & DBGP_ENABLED)
					? " IN USE"
					: "");
			if (!(temp & DBGP_ENABLED))
				ehci->debug = NULL;
		}
	}

	ehci_reset(ehci);

	/* at least the Genesys GL880S needs fixup here */
	temp = HCS_N_CC(ehci->hcs_params) * HCS_N_PCC(ehci->hcs_params);
	temp &= 0x0f;
	if (temp && HCS_N_PORTS(ehci->hcs_params) > temp) {
		ehci_dbg(ehci, "bogus port configuration: "
			"cc=%d x pcc=%d < ports=%d\n",
			HCS_N_CC(ehci->hcs_params),
			HCS_N_PCC(ehci->hcs_params),
			HCS_N_PORTS(ehci->hcs_params));

		switch (pdev->vendor) {
		case 0x17a0:		/* GENESYS */
			/* GL880S: should be PORTS=2 */
			temp |= (ehci->hcs_params & ~0xf);
			ehci->hcs_params = temp;
			break;
		case PCI_VENDOR_ID_NVIDIA:
			/* NF4: should be PCC=10 */
			break;
		}
	}

	/* Serial Bus Release Number is at PCI 0x60 offset */
	pci_read_config_byte(pdev, 0x60, &ehci->sbrn);

	/* Keep this around for a while just in case some EHCI
	 * implementation uses legacy PCI PM support.  This test
	 * can be removed on 17 Dec 2009 if the dev_warn() hasn't
	 * been triggered by then.
	 */
	if (!device_can_wakeup(&pdev->dev)) {
		u16	port_wake;

		pci_read_config_word(pdev, 0x62, &port_wake);
		if (port_wake & 0x0001) {
			dev_warn(&pdev->dev, "Enabling legacy PCI PM\n");
			device_set_wakeup_capable(&pdev->dev, 1);
		}
	}

#ifdef	CONFIG_USB_SUSPEND
	/* REVISIT: the controller works fine for wakeup iff the root hub
	 * itself is "globally" suspended, but usbcore currently doesn't
	 * understand such things.
	 *
	 * System suspend currently expects to be able to suspend the entire
	 * device tree, device-at-a-time.  If we failed selective suspend
	 * reports, system suspend would fail; so the root hub code must claim
	 * success.  That's lying to usbcore, and it matters for runtime
	 * PM scenarios with selective suspend and remote wakeup...
	 */
	if (ehci->no_selective_suspend && device_can_wakeup(&pdev->dev))
		ehci_warn(ehci, "selective suspend/wakeup unavailable\n");
#endif

	ehci_port_power(ehci, 1);
	retval = ehci_pci_reinit(ehci, pdev);
done:
	return retval;
}

/*-------------------------------------------------------------------------*/

#ifdef	CONFIG_PM

/* suspend/resume, section 4.3 */

/* These routines rely on the PCI bus glue
 * to handle powerdown and wakeup, and currently also on
 * transceivers that don't need any software attention to set up
 * the right sort of wakeup.
 * Also they depend on separate root hub suspend/resume.
 */

static int ehci_pci_suspend(struct usb_hcd *hcd, bool do_wakeup)
{
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	unsigned long		flags;
	int			rc = 0;
	int			port;

	if (time_before(jiffies, ehci->next_statechange))
		msleep(10);

	/* s0i3 may poweroff SRAM, backup the SRAM */
	if (hcd->has_sram && sram_backup(hcd)) {
		ehci_warn(ehci, "sram_backup failed\n");
		return -EPERM;
	}

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible.  The PM and USB cores make sure that
	 * the root hub is either suspended or stopped.
	 */
	ehci_prepare_ports_for_controller_suspend(ehci, do_wakeup);
	spin_lock_irqsave (&ehci->lock, flags);
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	(void)ehci_readl(ehci, &ehci->regs->intr_enable);

	/* Set HOSTPC_PHCD if not set yet to let PHY enter low-power mode */
	if (ehci->has_hostpc) {
		spin_unlock_irqrestore(&ehci->lock, flags);
		usleep_range(5000, 6000);
		spin_lock_irqsave(&ehci->lock, flags);

		port = HCS_N_PORTS(ehci->hcs_params);
		while (port--) {
			u32 __iomem	*hostpc_reg;
			u32		temp;

			hostpc_reg = (u32 __iomem *)((u8 *) ehci->regs
					 + HOSTPC0 + 4 * port);
			temp = ehci_readl(ehci, hostpc_reg);

			if (!(temp & HOSTPC_PHCD))
				ehci_writel(ehci, temp | HOSTPC_PHCD,
						hostpc_reg);
			temp = ehci_readl(ehci, hostpc_reg);
			ehci_dbg(ehci, "Port %d PHY low-power mode %s\n",
					port, (temp & HOSTPC_PHCD) ?
					"succeeded" : "failed");
		}
	}

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	spin_unlock_irqrestore (&ehci->lock, flags);

	// could save FLADJ in case of Vaux power loss
	// ... we'd only use it to handle clock skew

	return rc;
}

static bool usb_is_intel_switchable_ehci(struct pci_dev *pdev)
{
	return pdev->class == PCI_CLASS_SERIAL_USB_EHCI &&
		pdev->vendor == PCI_VENDOR_ID_INTEL &&
		(pdev->device == 0x1E26 ||
		 pdev->device == 0x8C2D ||
		 pdev->device == 0x8C26);
}

static void ehci_enable_xhci_companion(void)
{
	struct pci_dev		*companion = NULL;

	/* The xHCI and EHCI controllers are not on the same PCI slot */
	for_each_pci_dev(companion) {
		if (!usb_is_intel_switchable_xhci(companion))
			continue;
		usb_enable_xhci_ports(companion);
		return;
	}
}

static int ehci_pci_resume(struct usb_hcd *hcd, bool hibernated)
{
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	struct pci_dev		*pdev = to_pci_dev(hcd->self.controller);

	/* s0i3 may have poweroff the SRAM, restore it here*/
	if (hcd->has_sram && sram_restore(hcd)) {
		ehci_warn(ehci, "sram_restore failed, stop resuming.\n");
		return -EPERM;
	}

	/* The BIOS on systems with the Intel Panther Point chipset may or may
	 * not support xHCI natively.  That means that during system resume, it
	 * may switch the ports back to EHCI so that users can use their
	 * keyboard to select a kernel from GRUB after resume from hibernate.
	 *
	 * The BIOS is supposed to remember whether the OS had xHCI ports
	 * enabled before resume, and switch the ports back to xHCI when the
	 * BIOS/OS semaphore is written, but we all know we can't trust BIOS
	 * writers.
	 *
	 * Unconditionally switch the ports back to xHCI after a system resume.
	 * We can't tell whether the EHCI or xHCI controller will be resumed
	 * first, so we have to do the port switchover in both drivers.  Writing
	 * a '1' to the port switchover registers should have no effect if the
	 * port was already switched over.
	 */
	if (usb_is_intel_switchable_ehci(pdev))
		ehci_enable_xhci_companion();

	// maybe restore FLADJ

	if (time_before(jiffies, ehci->next_statechange))
		msleep(100);

	/* Mark hardware accessible again as we are out of D3 state by now */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	/* If CF is still set and we aren't resuming from hibernation
	 * then we maintained PCI Vaux power.
	 * Just undo the effect of ehci_pci_suspend().
	 */
	if (ehci_readl(ehci, &ehci->regs->configured_flag) == FLAG_CF &&
				!hibernated) {
		int	mask = INTR_MASK;

		ehci_prepare_ports_for_controller_resume(ehci);
		if (!hcd->self.root_hub->do_remote_wakeup)
			mask &= ~STS_PCD;
		ehci_writel(ehci, mask, &ehci->regs->intr_enable);
		ehci_readl(ehci, &ehci->regs->intr_enable);
		return 0;
	}

	usb_root_hub_lost_power(hcd->self.root_hub);

	/* Else reset, to cope with power loss or flush-to-storage
	 * style "resume" having let BIOS kick in during reboot.
	 */
	(void) ehci_halt(ehci);
	(void) ehci_reset(ehci);
	(void) ehci_pci_reinit(ehci, pdev);

	/* emptying the schedule aborts any urbs */
	spin_lock_irq(&ehci->lock);
	if (ehci->reclaim)
		end_unlink_async(ehci);
	ehci_work(ehci);
	spin_unlock_irq(&ehci->lock);

	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command);	/* unblock posted writes */

	/* here we "know" root ports should always stay powered */
	ehci_port_power(ehci, 1);

	hcd->state = HC_STATE_SUSPENDED;
	return 0;
}
#endif

static int ehci_update_device(struct usb_hcd *hcd, struct usb_device *udev)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int rc = 0;

	if (!udev->parent) /* udev is root hub itself, impossible */
		rc = -1;
	/* we only support lpm device connected to root hub yet */
	if (ehci->has_lpm && !udev->parent->parent) {
		rc = ehci_lpm_set_da(ehci, udev->devnum, udev->portnum);
		if (!rc)
			rc = ehci_lpm_check(ehci, udev->portnum);
	}
	return rc;
}

static const struct hc_driver ehci_pci_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"EHCI Host Controller",
	.hcd_priv_size =	sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ehci_irq,
	.flags =		HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset =		ehci_pci_setup,
	.start =		ehci_run,
#ifdef	CONFIG_PM
	.pci_suspend =		ehci_pci_suspend,
	.pci_resume =		ehci_pci_resume,
#endif
	.stop =			ehci_stop,
	.shutdown =		ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ehci_urb_enqueue,
	.urb_dequeue =		ehci_urb_dequeue,
	.endpoint_disable =	ehci_endpoint_disable,
	.endpoint_reset =	ehci_endpoint_reset,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ehci_hub_status_data,
	.hub_control =		ehci_hub_control,
	.bus_suspend =		ehci_bus_suspend,
	.bus_resume =		ehci_bus_resume,
	.relinquish_port =	ehci_relinquish_port,
	.port_handed_over =	ehci_port_handed_over,

	/*
	 * call back when device connected and addressed
	 */
	.update_device =	ehci_update_device,

	.clear_tt_buffer_complete	= ehci_clear_tt_buffer_complete,
};

/*-------------------------------------------------------------------------*/

/* PCI driver selection metadata; PCI hotplugging uses this */
static const struct pci_device_id pci_ids [] = { {
	/* handle any USB 2.0 EHCI controller */
	PCI_DEVICE_CLASS(PCI_CLASS_SERIAL_USB_EHCI, ~0),
	.driver_data =	(unsigned long) &ehci_pci_hc_driver,
	},
	{ /* end: all zeroes */ }
};
MODULE_DEVICE_TABLE(pci, pci_ids);

/* pci driver glue; this is a "new style" PCI driver module */
static struct pci_driver ehci_pci_driver = {
	.name =		(char *) hcd_name,
	.id_table =	pci_ids,

	.probe =	usb_hcd_pci_probe,
	.remove =	usb_hcd_pci_remove,
	.shutdown = 	usb_hcd_pci_shutdown,

#ifdef CONFIG_PM_SLEEP
	.driver =	{
		.pm =	&usb_hcd_pci_pm_ops
	},
#endif
};

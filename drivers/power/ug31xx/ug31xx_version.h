#define UG31XX_DRIVER_VERSION       (110)
#define UG31XX_DRIVER_VERSION_STR   ("110")
#define UG31XX_DRIVER_RELEASE_DATE  ("20140710-070130")
#define UG31XX_DRIVER_RELEASE_NOTE  ("645:6")

/// Release Note
///
///   1. Add coulomb counter checking criterion at power on if system time overflown
///   2. Report driver status at POWER_SUPPLY_PROP_TECHNOLOGY, Unknown for not ready
///   3. Fix the bug of initial battery look-up table range
///   4. Check the cable status after initial stage

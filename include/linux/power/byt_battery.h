#ifndef __BYT_BATTERY_H_
#define __BYT_BATTERY_H_

//<asus-ych20130904>

#pragma pack(1)
struct byt_battery 
{
	u16	Control;
	u16	AtRate;
	u16	AtRateTimeToEmpty;
	u16	Temperature;
	u16	Voltage;
	u16	Flags;
	u16	NominalAvailableCapacity;
	u16	FullAvailableCapacity;
	u16	RemainingCapacity; //0x10
	u16	FullChargeCapacity;
	u16	AverageCurrent;
	u16	TimeToEmpty;
	u16	TimeToFull;
	u16	StandbyCurrent;
	u16	StandbyTimeToEmpty;
	u16	MaxLoadCurrent;
	u16	MaxLoadTimeToEmpty;//0x20
    	u16	AvailableEnergy;
    	u16	AveragePower;
    	u16	TimeToEmptyAtConstantPower;
	u16	DesignCapacity;
    	u16	CycleCount;
	u16	StateOfCharge;
    	u8	ChargerStatusRegister;
    	u8	ChargeCurrentLimit;
    	u32	BatteryTripPoint;
	u8	SkinTemp0;
	u16	SkinTemp0Aux0;
    	u16	SkinTemp0Aux1;
};
#pragma pack(0)


struct byt_platform_data {
	int gpio;
};


#endif 

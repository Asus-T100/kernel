/**
 * @filename  uG31xx_Platform.h
 *
 *  Define the platform for uG31xx driver
 *
 * @author  AllenTeng <allen_teng@upi-semi.com>
 * @revision  $Revision: 81 $
 */

#ifndef BUILD_UG31XX_LIB

  #ifndef uG31xx_OS_ANDROID

    #define uG31xx_OS_WINDOWS

  #endif  ///< end of uG31xx_OS_ANDROID
  
#endif  ///< end of BUILD_UG31XX_LIB

/// [AT-PM] : Used for ANDROID linux kernel ; 09/08/2013
//#define uG31xx_OS_ANDROID
//#define UG31XX_USE_SHELL_AP_FOR_FILE_OP
//#define UG31XX_USE_DAEMON_AP_FOR_FILE_OP
//#define UG31XX_SHELL_ALGORITHM
//#define ANDROID_SHELL_ALGORITHM
//#define UG31XX_CELL_REPLACE_TEST
//#define UG31XX_2_PERCENT_1_MIN
//#define UG31XX_ADC_NO_TEMP_COMPENSATION
//#define FEATURE_DISABLE_SUSPEND_OPERATION

/// [AT-PM] : Used for ANDROID boot code ; 09/08/2013
//#define uG31xx_BOOT_LOADER  
//#define uG31xx_ALIGNED_4
//#define uG31xx_NO_MEM_UNIT


/*******************************************************************************
    CANopen Object Dictionary definition for CANopenNode V4

    This file was automatically generated by CANopenEditor v4.0-51-g2d9b1ad

    https://github.com/CANopenNode/CANopenNode
    https://github.com/CANopenNode/CANopenEditor

    DON'T EDIT THIS FILE MANUALLY !!!!
********************************************************************************

    File info:
        File Names:   OD.h; OD.c
        Project File: KEYPAD.xdd
        File Version: 3

        Created:      31.01.2017 17:13:00
        Created By:   Smoke666
        Modified:     29.11.2021 12:13:25
        Modified By:  rOBIN 2

    Device Info:
        Vendor Name:  VENDORNAME
        Vendor ID:    2
        Product Name: Keypad
        Product ID:   1

        Description:  sdfsdsdf
*******************************************************************************/

#ifndef OD_H
#define OD_H
/*******************************************************************************
    Counters of OD objects
*******************************************************************************/
#define OD_CNT_NMT 1
#define OD_CNT_EM 1
#define OD_CNT_SYNC 1
#define OD_CNT_SYNC_PROD 1
#define OD_CNT_TIME 1
#define OD_CNT_EM_PROD 1
#define OD_CNT_HB_CONS 1
#define OD_CNT_HB_PROD 1
#define OD_CNT_RPDO 4
#define OD_CNT_TPDO 1


/*******************************************************************************
    Sizes of OD arrays
*******************************************************************************/
#define OD_CNT_ARR_1003 16
#define OD_CNT_ARR_1011 4
#define OD_CNT_ARR_1016 8
#define OD_CNT_ARR_2000 1
#define OD_CNT_ARR_2001 3
#define OD_CNT_ARR_2002 3
#define OD_CNT_ARR_2003 6


/*******************************************************************************
    OD data declaration of all groups
*******************************************************************************/
typedef struct {
    uint32_t x1000_deviceType;
    char x1008_manufacturerDeviceName[12];
    char x1009_manufacturerHardwareVersion[5];
    char x100A_manufacturerSoftwareVersion[5];
    struct {
        uint8_t maxSub_index;
        uint32_t vendor_ID;
        uint32_t productCode;
        uint32_t revisionNumber;
        uint32_t serialNumber;
    } x1018_identity;
    struct {
        uint8_t maxSub_index;
        uint32_t COB_IDUsedByRPDO;
        uint8_t transmissionType;
    } x1400_RPDOCommunicationParameter;
    struct {
        uint8_t maxSub_index;
        uint32_t COB_IDUsedByRPDO;
        uint8_t transmissionType;
    } x1401_RPDOCommunicationParameter;
    struct {
        uint8_t maxSub_index;
        uint32_t COB_IDUsedByRPDO;
        uint8_t transmissionType;
    } x1402_RPDOCommunicationParameter;
    struct {
        uint8_t maxSub_index;
        uint32_t COB_IDUsedByRPDO;
        uint8_t transmissionType;
    } x1403_RPDOCommunicationParameter;
    struct {
        uint8_t numberOfMappedObjects;
        uint32_t mappedObject_1;
        uint32_t mappedObject_2;
        uint32_t mappedObject_3;
    } x1600_RPDOMappingParameter;
    struct {
        uint8_t numberOfMappedObjects;
        uint32_t mappedObject_1;
        uint32_t mappedObject_2;
        uint32_t mappedObject_3;
    } x1601_RPDOMappingParameter;
    struct {
        uint8_t numberOfMappedObjects;
        uint32_t mappedObject_1;
    } x1602_RPDOMappingParameter;
    struct {
        uint8_t numberOfMappedObjects;
        uint32_t mappedObject_1;
    } x1603_RPDOMappingParameter;
    struct {
        uint8_t maxSub_index;
        uint32_t COB_IDUsedByTPDO;
        uint8_t transmissionType;
        uint16_t eventTimer;
    } x1800_TPDOCommunicationParameter;
    struct {
        uint8_t numberOfMappedObjects;
        uint32_t mappedObject_1;
        uint32_t mappedObject_2;
        uint32_t mappedObject_3;
        uint32_t mappedObject_4;
        uint32_t mappedObject_5;
    } x1A00_TPDOMappingParameter;
} OD_ROM_t;

typedef struct {
    uint8_t x1001_errorRegister;
    char x100B_object_100BhModelID[15];
    uint8_t x1011_restoreDefaultParameters_sub0;
    uint32_t x1011_restoreDefaultParameters[OD_CNT_ARR_1011];
    uint8_t x2000_digitalInputModuleKeysStates_sub0;
    uint8_t x2000_digitalInputModuleKeysStates[OD_CNT_ARR_2000];
    uint8_t x2001_digitalOutputModuleLED_ON_sub0;
    uint8_t x2001_digitalOutputModuleLED_ON[OD_CNT_ARR_2001];
    uint8_t x2002_digitalOutputModuleLEDBlink_sub0;
    uint8_t x2002_digitalOutputModuleLEDBlink[OD_CNT_ARR_2002];
    uint8_t x2003_digitalOutputModuleBrightnessLevel_sub0;
    uint8_t x2003_digitalOutputModuleBrightnessLevel[OD_CNT_ARR_2003];
    uint8_t x2013_CANopenNodeID;
    uint8_t x2014_setStartupLEDShow;
    uint8_t x2100_setDEMOMode;
} OD_RAM_t;

typedef struct {
    uint32_t x1005_COB_ID_SYNCMessage;
    uint32_t x1006_communicationCyclePeriod;
    uint32_t x1007_synchronousWindowLength;
    uint32_t x1012_COB_IDTimeStampObject;
    uint32_t x1014_COB_ID_EMCY;
    uint16_t x1015_inhibitTimeEMCY;
    uint8_t x1016_consumerHeartbeatTime_sub0;
    uint32_t x1016_consumerHeartbeatTime[OD_CNT_ARR_1016];
    uint16_t x1017_producerHeartbeatTime;
    uint8_t x1019_synchronousCounterOverflowValue;
} OD_PERSIST_COMM_t;

#ifndef OD_ATTR_ROM
#define OD_ATTR_ROM
#endif
extern OD_ATTR_ROM OD_ROM_t OD_ROM;

#ifndef OD_ATTR_RAM
#define OD_ATTR_RAM
#endif
extern OD_ATTR_RAM OD_RAM_t OD_RAM;

#ifndef OD_ATTR_PERSIST_COMM
#define OD_ATTR_PERSIST_COMM
#endif
extern OD_ATTR_PERSIST_COMM OD_PERSIST_COMM_t OD_PERSIST_COMM;

#ifndef OD_ATTR_OD
#define OD_ATTR_OD
#endif
extern OD_ATTR_OD OD_t *OD;


/*******************************************************************************
    Object dictionary entries - shortcuts
*******************************************************************************/
#define OD_ENTRY_H1000 &OD->list[0]
#define OD_ENTRY_H1001 &OD->list[1]
#define OD_ENTRY_H1003 &OD->list[2]
#define OD_ENTRY_H1005 &OD->list[3]
#define OD_ENTRY_H1006 &OD->list[4]
#define OD_ENTRY_H1007 &OD->list[5]
#define OD_ENTRY_H1008 &OD->list[6]
#define OD_ENTRY_H1009 &OD->list[7]
#define OD_ENTRY_H100A &OD->list[8]
#define OD_ENTRY_H100B &OD->list[9]
#define OD_ENTRY_H1011 &OD->list[10]
#define OD_ENTRY_H1012 &OD->list[11]
#define OD_ENTRY_H1014 &OD->list[12]
#define OD_ENTRY_H1015 &OD->list[13]
#define OD_ENTRY_H1016 &OD->list[14]
#define OD_ENTRY_H1017 &OD->list[15]
#define OD_ENTRY_H1018 &OD->list[16]
#define OD_ENTRY_H1019 &OD->list[17]
#define OD_ENTRY_H1400 &OD->list[18]
#define OD_ENTRY_H1401 &OD->list[19]
#define OD_ENTRY_H1402 &OD->list[20]
#define OD_ENTRY_H1403 &OD->list[21]
#define OD_ENTRY_H1600 &OD->list[22]
#define OD_ENTRY_H1601 &OD->list[23]
#define OD_ENTRY_H1602 &OD->list[24]
#define OD_ENTRY_H1603 &OD->list[25]
#define OD_ENTRY_H1800 &OD->list[26]
#define OD_ENTRY_H1A00 &OD->list[27]
#define OD_ENTRY_H2000 &OD->list[28]
#define OD_ENTRY_H2001 &OD->list[29]
#define OD_ENTRY_H2002 &OD->list[30]
#define OD_ENTRY_H2003 &OD->list[31]
#define OD_ENTRY_H2010 &OD->list[32]
#define OD_ENTRY_H2011 &OD->list[33]
#define OD_ENTRY_H2012 &OD->list[34]
#define OD_ENTRY_H2013 &OD->list[35]
#define OD_ENTRY_H2014 &OD->list[36]
#define OD_ENTRY_H2100 &OD->list[37]


/*******************************************************************************
    Object dictionary entries - shortcuts with names
*******************************************************************************/
#define OD_ENTRY_H1000_deviceType &OD->list[0]
#define OD_ENTRY_H1001_errorRegister &OD->list[1]
#define OD_ENTRY_H1003_pre_definedErrorField &OD->list[2]
#define OD_ENTRY_H1005_COB_ID_SYNCMessage &OD->list[3]
#define OD_ENTRY_H1006_communicationCyclePeriod &OD->list[4]
#define OD_ENTRY_H1007_synchronousWindowLength &OD->list[5]
#define OD_ENTRY_H1008_manufacturerDeviceName &OD->list[6]
#define OD_ENTRY_H1009_manufacturerHardwareVersion &OD->list[7]
#define OD_ENTRY_H100A_manufacturerSoftwareVersion &OD->list[8]
#define OD_ENTRY_H100B_object_100BhModelID &OD->list[9]
#define OD_ENTRY_H1011_restoreDefaultParameters &OD->list[10]
#define OD_ENTRY_H1012_COB_IDTimeStampObject &OD->list[11]
#define OD_ENTRY_H1014_COB_ID_EMCY &OD->list[12]
#define OD_ENTRY_H1015_inhibitTimeEMCY &OD->list[13]
#define OD_ENTRY_H1016_consumerHeartbeatTime &OD->list[14]
#define OD_ENTRY_H1017_producerHeartbeatTime &OD->list[15]
#define OD_ENTRY_H1018_identity &OD->list[16]
#define OD_ENTRY_H1019_synchronousCounterOverflowValue &OD->list[17]
#define OD_ENTRY_H1400_RPDOCommunicationParameter &OD->list[18]
#define OD_ENTRY_H1401_RPDOCommunicationParameter &OD->list[19]
#define OD_ENTRY_H1402_RPDOCommunicationParameter &OD->list[20]
#define OD_ENTRY_H1403_RPDOCommunicationParameter &OD->list[21]
#define OD_ENTRY_H1600_RPDOMappingParameter &OD->list[22]
#define OD_ENTRY_H1601_RPDOMappingParameter &OD->list[23]
#define OD_ENTRY_H1602_RPDOMappingParameter &OD->list[24]
#define OD_ENTRY_H1603_RPDOMappingParameter &OD->list[25]
#define OD_ENTRY_H1800_TPDOCommunicationParameter &OD->list[26]
#define OD_ENTRY_H1A00_TPDOMappingParameter &OD->list[27]
#define OD_ENTRY_H2000_digitalInputModuleKeysStates &OD->list[28]
#define OD_ENTRY_H2001_digitalOutputModuleLED_ON &OD->list[29]
#define OD_ENTRY_H2002_digitalOutputModuleLEDBlink &OD->list[30]
#define OD_ENTRY_H2003_digitalOutputModuleBrightnessLevel &OD->list[31]
#define OD_ENTRY_H2010_baudRateSetting &OD->list[32]
#define OD_ENTRY_H2011_setBoot_upService &OD->list[33]
#define OD_ENTRY_H2012_setDeviceActiveOnStartup &OD->list[34]
#define OD_ENTRY_H2013_CANopenNodeID &OD->list[35]
#define OD_ENTRY_H2014_setStartupLEDShow &OD->list[36]
#define OD_ENTRY_H2100_setDEMOMode &OD->list[37]


/*******************************************************************************
    OD config structure
*******************************************************************************/
#ifdef CO_MULTIPLE_OD
#define OD_INIT_CONFIG(config) {\
    (config).CNT_NMT = OD_CNT_NMT;\
    (config).ENTRY_H1017 = OD_ENTRY_H1017;\
    (config).CNT_HB_CONS = OD_CNT_HB_CONS;\
    (config).CNT_ARR_1016 = OD_CNT_ARR_1016;\
    (config).ENTRY_H1016 = OD_ENTRY_H1016;\
    (config).CNT_EM = OD_CNT_EM;\
    (config).ENTRY_H1001 = OD_ENTRY_H1001;\
    (config).ENTRY_H1014 = OD_ENTRY_H1014;\
    (config).ENTRY_H1015 = OD_ENTRY_H1015;\
    (config).CNT_ARR_1003 = OD_CNT_ARR_1003;\
    (config).ENTRY_H1003 = OD_ENTRY_H1003;\
    (config).CNT_SDO_SRV = 0;\
    (config).ENTRY_H1200 = OD_ENTRY_H1200;\
    (config).CNT_SDO_CLI = 0;\
    (config).ENTRY_H1280 = OD_ENTRY_H1280;\
    (config).CNT_TIME = OD_CNT_TIME;\
    (config).ENTRY_H1012 = OD_ENTRY_H1012;\
    (config).CNT_SYNC = OD_CNT_SYNC;\
    (config).ENTRY_H1005 = OD_ENTRY_H1005;\
    (config).ENTRY_H1006 = OD_ENTRY_H1006;\
    (config).ENTRY_H1007 = OD_ENTRY_H1007;\
    (config).ENTRY_H1019 = OD_ENTRY_H1019;\
    (config).CNT_RPDO = OD_CNT_RPDO;\
    (config).ENTRY_H1400 = OD_ENTRY_H1400;\
    (config).ENTRY_H1600 = OD_ENTRY_H1600;\
    (config).CNT_TPDO = OD_CNT_TPDO;\
    (config).ENTRY_H1800 = OD_ENTRY_H1800;\
    (config).ENTRY_H1A00 = OD_ENTRY_H1A00;\
    (config).CNT_LEDS = 0;\
    (config).CNT_GFC = 0;\
    (config).ENTRY_H1300 = NULL;\
    (config).CNT_SRDO = 0;\
    (config).ENTRY_H1301 = NULL;\
    (config).ENTRY_H1381 = NULL;\
    (config).ENTRY_H13FE = NULL;\
    (config).ENTRY_H13FF = NULL;\
    (config).CNT_LSS_SLV = 0;\
    (config).CNT_LSS_MST = 0;\
    (config).CNT_GTWA = 0;\
    (config).CNT_TRACE = 0;\
}
#endif

#endif /* OD_H */

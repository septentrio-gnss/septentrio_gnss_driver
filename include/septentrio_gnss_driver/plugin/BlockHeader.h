// Custom constructor macro to initalize SBF msgs with do_not_use values in block
// header
#define SEPTENTRIO_GNSS_DRIVER_MESSAGE_BLOCKHEADER_PLUGIN_CONSTRUCTOR               \
    BlockHeader_() :                                                                \
        sync_1(0x24), sync_2(0x40), crc(0), id(0), revision(0), length(0),          \
        tow(4294967295UL), wnc(65535)                                               \
    {                                                                               \
    }

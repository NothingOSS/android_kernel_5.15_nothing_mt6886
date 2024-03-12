#ifndef PANEL_RM692E5_FHDP_DSI_CMD
#define PANEL_RM692E5_FHDP_DSI_CMD


#define REGFLAG_DELAY           0xFFFC
#define REGFLAG_UDELAY          0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW       0xFFFE
#define REGFLAG_RESET_HIGH      0xFFFF

#define FRAME_WIDTH                 1080
#define FRAME_HEIGHT                2400

#define PHYSICAL_WIDTH              70697
#define PHYSICAL_HEIGHT             157104

#define DATA_RATE                   373*2
#define HSA                         39
#define HBP                         36
#define VSA                         21
#define VBP                         16

/*Parameter setting for mode 0 Start*/
#define MODE_0_FPS                  60
#define MODE_0_VFP                  16
#define MODE_0_HFP                  26
#define MODE_0_DATA_RATE            373*2
/*Parameter setting for mode 0 End*/

/*Parameter setting for mode 1 Start*/
#define MODE_1_FPS                  90
#define MODE_1_VFP                  16
#define MODE_1_HFP                  26
#define MODE_1_DATA_RATE            250*2
/*Parameter setting for mode 1 End*/

/*Parameter setting for mode 2 Start*/
#define MODE_2_FPS                  120
#define MODE_2_VFP                  16
#define MODE_2_HFP                  26
#define MODE_2_DATA_RATE            760 //1004 prize
/*Parameter setting for mode 2 End*/

#define LFR_EN                      1
/* DSC RELATED */

#define DSC_ENABLE                  0
#define DSC_VER                     17
#define DSC_SLICE_MODE              1
#define DSC_RGB_SWAP                0
#define DSC_DSC_CFG                 34
#define DSC_RCT_ON                  1
#define DSC_BIT_PER_CHANNEL         8
#define DSC_DSC_LINE_BUF_DEPTH      11
#define DSC_BP_ENABLE               1
#define DSC_BIT_PER_PIXEL           128
//define DSC_PIC_HEIGHT
//define DSC_PIC_WIDTH
#define DSC_SLICE_HEIGHT            20
#define DSC_SLICE_WIDTH             540
#define DSC_CHUNK_SIZE              540
#define DSC_XMIT_DELAY              512
#define DSC_DEC_DELAY               526
#define DSC_SCALE_VALUE             32
#define DSC_INCREMENT_INTERVAL      488
#define DSC_DECREMENT_INTERVAL      7
#define DSC_LINE_BPG_OFFSET         12
#define DSC_NFL_BPG_OFFSET          1294
#define DSC_SLICE_BPG_OFFSET        1302
#define DSC_INITIAL_OFFSET          6144
#define DSC_FINAL_OFFSET            4336
#define DSC_FLATNESS_MINQP          7
#define DSC_FLATNESS_MAXQP          16
#define DSC_RC_MODEL_SIZE           8192
#define DSC_RC_EDGE_FACTOR          6
#define DSC_RC_QUANT_INCR_LIMIT0    15
#define DSC_RC_QUANT_INCR_LIMIT1    15
#define DSC_RC_TGT_OFFSET_HI        3
#define DSC_RC_TGT_OFFSET_LO        3

#endif //end of PANEL_RM692E5_FHDP_DSI_CMD

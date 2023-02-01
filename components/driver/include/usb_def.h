#ifndef _USB_DEF_H_
#define _USB_DEF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bus_dev.h"

#define OTG_PCD_IN_FIFO_BASE                 (USB_BASE_ADDR + 0x00000UL)
#define OTG_PCD_OUT_FIFO_BASE                (USB_BASE_ADDR + 0x40000UL)
#define OTG_PCD_IN_EP_REG_BASE               (USB_BASE_ADDR + 0x44000UL)
#define OTG_PCD_OUT_EP_REG_BASE              (USB_BASE_ADDR + 0x44100UL)
#define OTG_PCD_GLB_REG_ADDR                 (USB_BASE_ADDR + 0x44200UL)
#define OTG_PCD_MAC_REG_ADDR                 (USB_BASE_ADDR + 0x44400UL)
#define OTG_HCD_GLB_REG_ADDR                 (USB_BASE_ADDR + 0x44800UL)
#define OTG_HCD_MAC_REG_ADDR                 (USB_BASE_ADDR + 0x44C00UL)
#define OTG_DIU_REG_ADDR                     (USB_BASE_ADDR + 0x45000UL)
#define OTG_TOP_REG_ADDR                     (USB_BASE_ADDR + 0x80000UL)

#define OTG_PCD_IN_EP_FIFO(__EP_NUM__)       (OTG_PCD_IN_FIFO_BASE + (__EP_NUM__)*0x4000)
#define OTG_PCD_OUT_EP_FIFO(__EP_NUM__)      (OTG_PCD_OUT_FIFO_BASE)
#define OTG_PCD_IN_EP_REG(__EP_NUM__)        (OTG_PCD_IN_EP_REG_BASE + (__EP_NUM__)*16)
#define OTG_PCD_OUT_EP_REG(__EP_NUM__)       (OTG_PCD_OUT_EP_REG_BASE + (__EP_NUM__)*16)
#define OTG_PCD_EP_MAC_REG(__EP_NUM__)       (OTG_PCD_MAC_REG_BASE + (__EP_NUM__)*4)

#define USB_LS_SPEED_MODE                            (0x2)
#define USB_FS_SPEED_MODE                            (0x3)

#define USB_REMOTE_WAKEUP_SUPOORT                    (0x1)
#define USB_REMOTE_WAKEUP_UNSUPPORT                  (0x0)

#define USB_CSR_SELF_POWERED                         (0x1)
#define USB_CSR_BUS_POWERED                          (0x0)

#define USB_SYNC_FRAME_SUPPORT                       (0x1)
#define USB_SYNC_FRAME_UNSUPPORT                     (0x0)

#define USB_CSR_PRG_SUPPORT                          (0x1)
#define USB_CSR_PRG_UNSUPPORT                        (0x0)

#define USB_CSR_INIT_DONE                            (0x1)
#define USB_CSR_INIT_IN_PROGRESS                     (0x0)

#define USB_SET_DESC_CMD_SUPPORT                     (0x1)
#define USB_SET_DESC_CMD_UNSUPPORT                   (0x0)

#define USB_DEVICE_MODE                              (0x1)
#define USB_HOST_MODE                                (0x0)

#define USB_SCALE_DOWN_ENABLE                        (0x1)
#define USB_SCALE_DOWN_DISABLE                       (0x0)

/*Device Interrupt */
#define USB_INT_ENABLE                               (0x1)
#define USB_INT_DISABLE                              (0x0)

/*FIFO Stauts */
#define USB_OUT_RX_FIFO_FAILURE                      (0x0)
#define USB_OUT_RX_FIFO_SUCCESS                      (0x1)

#define USB_OUT_TRANSFER_COMPLETE                    (0x1)
#define USB_OUT_TRANSFER_UNCOMPLETE                  (0x0)


#define BIT_SEND_STALL  BIT(0)
#define BIT_SEND_NAK    BIT(1)
#define BIT_FLUSH_FIFO  BIT(2)
#define BIT_FIFO_READY  BIT(3)

typedef union{
	uint32_t d32;
	struct{
		uint32_t SEND_STALL  : 1;
		uint32_t SEND_NAK    : 1;
		uint32_t FLUSH_FIFO  : 1;
		uint32_t FIFO_READY  : 1;
	}b;
}ep_in_ctl_t;
typedef union{
	uint32_t d32;
	struct{
		uint32_t STATUS            : 1;
		uint32_t DATA_SENT         : 1;
		uint32_t BELOW_THD         : 1;
		uint32_t NAK_SENT          : 1;
		uint32_t DATA_UNDERRUN_ERR : 1;
		uint32_t ISO_TX_DONE       : 1;
	}b;
}ep_in_sts_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t SEND_STALL      : 1;
		uint32_t RSVD            : 1;
		uint32_t FLUSH_FIFO      : 1;
		uint32_t FIFO_READY      : 1;
	}b;
}ep_out_ctl_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t SPEED        : 2;
		uint32_t RMT_WKUP     : 1;
		uint32_t SELF_PWR     : 1;
		uint32_t SYNC_FRAME   : 1;
		uint32_t CSR_PRG_SUP  : 1;
		uint32_t CSR_DONE     : 1;
		uint32_t SET_DESC_SUP : 1;
		uint32_t HST_MODE     : 1;
		uint32_t SCALE_DOWN   : 1;
		uint32_t RSVD         : 1;
		uint32_t STATUS       : 2;
	}b;
}dev_cfg_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t CFG   : 4;
		uint32_t INTF  : 4;
		uint32_t ALT   : 4;
		uint32_t SUSP  : 1;
		uint32_t RSVD  : 8;
		uint32_t TS    : 11;
	}b;
}dev_sts_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t SC_INT_STS     : 1;
		uint32_t SI_INT_STS     : 1;
		uint32_t RSVD           : 1;
		uint32_t UR_INT_STS     : 1;
		uint32_t US_INT_STS     : 1;
		uint32_t SOF_INT_STS    : 1;
		uint32_t SETUP_INT_STS  : 1;
		uint32_t OUT_INT_STS    : 1;
		uint32_t PORT_INT_STS   : 1;
		uint32_t OTG_INT_STS    : 1;
		uint32_t I2C_INT_STS    : 1;
	}b;
}dev_int_sts_t;


#define SC_INT_BIT      BIT(0)
#define SI_INT_BIT      BIT(1)
//#define RSVD         BIT(2)
#define UR_INT_BIT      BIT(3)
#define US_INT_BIT      BIT(4)
#define SOF_INT_BIT     BIT(5)
#define SETUP_INT_BIT   BIT(6)
#define OUT_INT_BIT     BIT(7)
#define PORT_INT_BIT    BIT(8)
#define OTG_INT_BIT     BIT(9)
#define I2C_INT_BIT     BIT(10)

typedef union{
	uint32_t d32;
	struct{
		uint32_t SC_INT_EN      : 1;
		uint32_t SI_INT_EN      : 1;
		uint32_t RSVD           : 1;
		uint32_t UR_INT_EN      : 1;
		uint32_t US_INT_EN      : 1;
		uint32_t SOF_INT_EN     : 1;
		uint32_t SETUP_INT_EN   : 1;
		uint32_t OUT_INT_EN     : 1;
		uint32_t PORT_INT_EN    : 1;
		uint32_t OTG_INT_EN     : 1;
		uint32_t I2C_INT_EN     : 1;
	}b;
}dev_int_en_t;

typedef union {
	uint32_t d32;
	struct{
		 uint32_t IN_EP_INT_EN  : 16;
		 uint32_t OUT_EP_INT_EN : 16;
	}b;
}dev_glb_ep_int_en_t;

typedef union {
	uint32_t d32;
	struct{
		 uint32_t IN_EP_INT     : 16;
		 uint32_t OUT_EP_INT    : 16;
	}b;
}dev_glb_ep_int_sts_t;


typedef union{
	uint32_t d32;
	struct{
		 uint32_t TX_THD        : 10;
		 uint32_t RSVD          : 6;
		 uint32_t RX_THD        : 10;
	}b;
}dev_rx_tx_thd_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t STATUS           : 1;
		uint32_t STATUS_COMP      : 1;
		uint32_t ABOVE_THD        : 1;
		uint32_t RSVD0            : 1;
		uint32_t DATA_OVERRUN_ERR : 1;
		uint32_t RSVD1            : 11;
		uint32_t EP_NUM           : 4;
		uint32_t RSVD2            : 2;
		uint32_t TRANSFER_SZ      : 10;
	}b;
}dev_out_ep_fifo_sts_t;	//0x4_421C

typedef union{
	uint32_t d32;
	struct{
		uint32_t STATUS           : 1;
		uint32_t STATUS_COMP      : 1;
		uint32_t RSVD1            : 13;
		uint32_t SETUP_AFTER_OUT  : 1;
		uint32_t EP_NUM      : 4;
	}b;
}dev_setup_fifo_sts_t;	//0x4_4220

typedef	union{
	uint32_t d32;
	struct{
		uint32_t IN_EP_INT         : 16;
		uint32_t OUT_EP_INT         : 16;
	}b;
}dev_glb_ep_int_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t EP_NUM  : 4;
		uint32_t EP_DIR  : 1;
		uint32_t EP_TYPE : 2;
		uint32_t CFG_NUM : 4;
		uint32_t ITF_NUM : 4;
		uint32_t ALT_SET : 4;
		uint32_t MAX_PKT : 10;
	}b;
}dev_ep_info_t;	//0x4_4404 + x*4

typedef union{
	uint32_t d32;
	struct{
		uint32_t STATUS_INT    : 1;
		uint32_t ABOVE_THD_INT : 1;
		uint32_t BELOW_THD_INT : 1;
		uint32_t SOF_DUE_INT   : 1;
		uint32_t RSVD          : 4;
		uint32_t PORT_INT      : 1;
		uint32_t OTG_INT       : 1;
	}b;
}hst_int_sts_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t STATUS_INT_EN    : 1;
		uint32_t ABOVE_THD_INT_EN : 1;
		uint32_t BELOW_THD_INT_EN : 1;
		uint32_t SOF_DUE_INT_EN   : 1;
		uint32_t RSVD             : 4;
		uint32_t PORT_INT_EN      : 1;
		uint32_t OTG_INT_EN       : 1;
	}b;
}hst_int_en_t;

typedef struct{
	uint32_t d32;
	struct{
		uint32_t RESP_CODE         : 4;
		uint32_t TRANS_SZ          : 10;
	}b;
}hst_sts_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t FLUSH_FIFO       : 1;
		uint32_t RSVD             : 7;
		uint32_t SCALE_DOWN       : 1;
	}b;
}hst_ctl_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t RX_FIFO_SZ       : 10;
		uint32_t RSVD             : 6;
		uint32_t TX_FIFO_SZ       : 10;
	}b;
}hst_fifo_sz_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t RX_THD          : 10;
		uint32_t RSVD            : 6;
		uint32_t TX_THD          : 10;
	}b;
}hst_thd_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t CURCS          : 1;
		uint32_t PORTENSTA      : 1;
		uint32_t PORTSUSPSTA    : 1;
		uint32_t PORTOVCI       : 1;
		uint32_t PORTRSTSTA     : 1;
		uint32_t RSVD0          : 3;
		uint32_t PORTPWRSTA     : 1;
		uint32_t LOWSPDDEVATCH  : 1;
		uint32_t RSVD1          : 6;
		uint32_t CONSTACHG      : 1;
		uint32_t PORTENSTACHG   : 1;
		uint32_t PORTSUSPSTACHG : 1;
		uint32_t PORTOVCIC      : 1;
		uint32_t PORTRSTSCHG    : 1;
	}b;
}otg_hst_port_sts_ctl_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t ADDR       : 7;
		uint32_t EP_NUM     : 8;
		uint32_t DATA_PID   : 2;
		uint32_t TOKEN_TYPE : 4;
		uint32_t ISO        : 1;
		uint32_t SPEED      : 2;
		uint32_t TRANS_SZ   : 10;
	}b;
}otg_hst_token_t;

typedef struct{
	uint32_t d32;
	struct{
		uint32_t SRSUCC        : 1;
		uint32_t SRSCHG        : 1;
		uint32_t HNSUCC        : 1;
		uint32_t HNSSCHG       : 1;
		uint32_t SRD           : 1;
		uint32_t SRDSC         : 1;
		uint32_t HND           : 1;
		uint32_t HNDSC         : 1;
		uint32_t IDSTS         : 1;
		uint32_t HSTMOD        : 1;
		uint32_t SR            : 1;
		uint32_t HNPR          : 1;
		uint32_t HSHNPEN       : 1;
		uint32_t HPNEN         : 1;
		uint32_t SRPCAP        : 1;
		uint32_t HNPCAP        : 1;
		uint32_t RSVD          : 10;
		uint32_t OTGSI         : 1;
	}b;
}otg_ctl_sts_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t EP_ACTIVE			: 1;
		uint32_t LAST_DESC_LAST_PKT: 1;
		uint32_t DMA_BURST_SZ      : 4;
		uint32_t REQ_SIG_MAP       : 5;
		uint32_t EP_RST            : 1;
		uint32_t TRANS_TYPE        : 2;
		uint32_t DATA_SENT_INT_EN  : 1;
		uint32_t ISO_TX_DONE       : 1;
		uint32_t TRANS_SZ          : 16;
	}b;
}otg_dma_cfg_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t DM_PD_EN 			:1;
		uint32_t DM_PU_EN   		:1;
		uint32_t RSVD       		:29;
		uint32_t DM_FORCE_PULL_EN 	: 1;
	}b;
}otg_dm_pull_cfg_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t DP_PD_EN 			:1;
		uint32_t DP_PU_EN   		:1;
		uint32_t RSVD       		:29;
		uint32_t DP_FORCE_PULL_EN 	:1;
	}b;
}otg_dp_pull_cfg_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t DM_OE_N            :1;
		uint32_t DM_O               :1;
		uint32_t DM_I               :1;
		uint32_t RSVD               :13;
		uint32_t FORCE_DM_OE_EN     :1;
		uint32_t FORCE_DM_O_EN      :1;
		uint32_t FORCE_DM_I_EN      :1;
	}b;
}otg_dm_ctl_cfg_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t DP_OE_N            :1;
		uint32_t DP_O               :1;
		uint32_t DP_I               :1;
		uint32_t RSVD               :13;
		uint32_t FORCE_DP_OE_EN     :1;
		uint32_t FORCE_DP_O_EN      :1;
		uint32_t FORCE_DP_I_EN      :1;
	}b;
}otg_dp_ctl_cfg_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t RAW_VBUS_VALID     :1;
		uint32_t RAW_SESSION_VALID  :1;
		uint32_t RAW_ID_STATUS      :1;
		uint32_t RAW_OVERCURRENT    :1;
		uint32_t RSVD               :12;
		uint32_t VBUS_VALID_POST    :1;
		uint32_t SESSION_VALID_POST :1;
		uint32_t ID_STATUS_POST     :1;
		uint32_t OVERCURRENT_POST   :1;
		uint32_t DP_OUTPUT_FROM_OTG :1;
		uint32_t DM_OUTPUT_FROM_OTG :1;
		uint32_t DP_INPUT_TO_PHY    :1;
		uint32_t DM_INPUT_TO_PHY    :1;
		uint32_t DP_INPUT_FROM_PHY  :1;
		uint32_t DM_INPUT_FROM_PHY  :1;
	}b;
}otg_phy_status_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t VBUSVLD              :1;
		uint32_t SESSVLD              :1;
		uint32_t IDSTATUS             :1;
		uint32_t OVERCURRENT          :1;
		uint32_t RESUME               :1;
		uint32_t RSVD                 :11;
		uint32_t FORCE_VBUS_VLID_EN   :1;
		uint32_t FORCE_SESSION_VLD_EN :1;
		uint32_t FORCE_ID_STATUS_EN   :1;
		uint32_t FORCE_OVER_CURRENT_EN:1;
		uint32_t FORCE_RESUME_EN      :1;
	}b;
}otg_phy_ctl_t;


typedef union{
	uint32_t d32;
	struct{
		uint32_t RAW_SUSPEND          :1;
		uint32_t RAW_SPEED            :1;
		uint32_t RAW_VBUS_DRIVE       :1;
		uint32_t RAW_VBUS_CHARGE      :1;
		uint32_t RAW_VBUS_DISCHARGE   :1;
		uint32_t RAW_HNP_STATE        :1;
		uint32_t OTG_MODE             :1;
		uint32_t TXSE_STATE           :1;
		uint32_t RSVD                 :8;
		uint32_t SUSPEND_POST         :1;
		uint32_t SPEED_POST           :1;
		uint32_t VBUS_DRIVE_POST      :1;
		uint32_t VBUS_CHARGE_POST     :1;
		uint32_t VBUS_DISCHARGE_POST  :1;
	}b;
}otg_status_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t SUSPEND              :1;
		uint32_t SPEED                :1;
		uint32_t VBUS_DRIVE           :1;
		uint32_t VBUS_CHARGE          :1;
		uint32_t VBUS_DISCHARGE       :1;
		uint32_t FORCE_SUSPEND_EN     :1;
		uint32_t FORCE_SPEED_EN       :1;
		uint32_t FORCE_VBUS_DRIVE     :1;
		uint32_t FORCE_VBUS_CHARGE    :1;
		uint32_t FORCE_VBUS_DISCHARGE :1;
	}b;
}otg_ctl0_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t APP_RESUME_DEV       :1;
		uint32_t RSVD                 :31;
	}b;
}otg_ctl1_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t OTG_INT_SENSE        :1;
		uint32_t OTG_INT_LEVEL        :1;
		uint32_t RSVD                 :6;
		uint32_t HNP_INT              :1;
		uint32_t SRP_INT              :1;
		uint32_t VBUSVLDRISING_INT    :1;
		uint32_t OVERCURRENT_INT      :1;
	}b;
}otg_int_status_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t OTG_INT_SENSE_EN     :1;
		uint32_t OTG_INT_LEVEL_EN     :1;
		uint32_t RSVD                 :6;
		uint32_t HNP_INT_EN           :1;
		uint32_t SRP_INT_EN           :1;
		uint32_t VBUSVLDRISING_INT_EN :1;
		uint32_t OVERCURRENT_INT_EN   :1;
	}b;
}otg_int_en_t;

typedef union{

	uint32_t d32;
	struct{
		uint32_t ID_PD                :1;
		uint32_t ID_PU                :1;
	}b;
}id_pull_t;

typedef union{
	uint32_t d32;
	struct {
		uint32_t ID_OE                :1;
		uint32_t ID_O                 :1;
	}b;
}id_ctl_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t OTG_12M_EN           :1;
		uint32_t OTG_48M_EN           :1;
	}b;
}otg_clk_cfg_t;

typedef union{
	uint32_t d32;
	struct{
		uint32_t OTG_12M_FORCE_RST   :1;
		uint32_t OTG_PLL_FORCE_RST   :1;
		uint32_t OTG_48M_FORCE_RST   :1;
	}b;
}otg_rst_cfg_t;


#ifdef __cplusplus
};
#endif

#endif

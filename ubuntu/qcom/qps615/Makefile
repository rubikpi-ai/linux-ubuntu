ifeq ($(RELEASE_PACKAGE),1)
EXTRA_CFLAGS+=-DRELEASE_PACKAGE
endif
LBITS := $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
CCFLAGS += -m64
EXTRA_CFLAGS+=-DSYSTEM_IS_64
else
CCFLAGS += -m32
endif

EXTRA_CFLAGS+=-DTC956X
EXTRA_CFLAGS+=-DCONFIG_TC956X_PLATFORM_SUPPORT

ifeq ($(TC956X_PCIE_GEN3_SETTING),1)
EXTRA_CFLAGS+=-DTC956X_PCIE_GEN3_SETTING
endif

ifeq ($(TC956X_LOAD_FW_HEADER),1)
EXTRA_CFLAGS+=-DTC956X_LOAD_FW_HEADER
endif

ifeq ($(TC956X_IOCTL_REG_RD_WR_ENABLE),1)
EXTRA_CFLAGS+=-DTC956X_IOCTL_REG_RD_WR_ENABLE
endif

ifeq ($(TC956X_MAGIC_PACKET_WOL_GPIO),1)
EXTRA_CFLAGS+=-DTC956X_MAGIC_PACKET_WOL_GPIO
endif

ifeq ($(TC956X_MAGIC_PACKET_WOL_CONF),1)
EXTRA_CFLAGS+=-DTC956X_MAGIC_PACKET_WOL_CONF
endif

ifeq ($(TC956X_DMA_OFFLOAD_ENABLE),1)
EXTRA_CFLAGS+=-DTC956X_DMA_OFFLOAD_ENABLE
DMA_OFFLOAD = 1
endif

ifeq ($(vf), 1)
EXTRA_CFLAGS+=-DTC956X_SRIOV_VF

obj-m := tc956x_vf_pcie_eth.o
tc956x_vf_pcie_eth-y := tc956xmac_main.o tc956xmac_ethtool.o \
	      mmc_core.o tc956xmac_hwtstamp.o tc956xmac_ptp.o \
	      hwif.o  tc956xmac_tc.o dwxgmac2_core.o \
	      dwxgmac2_descs.o dwxgmac2_dma.o tc956x_pci.o \
	      tc956x_vf_mbx_wrapper.o tc956x_msigen.o \
	      tc956x_vf_rsc_mng.o tc956x_vf_mbx.o \
	      tc956x_pcie_logstat.o tc956x_qcom.o
	      
else

EXTRA_CFLAGS+=-DTC956X_SRIOV_PF

obj-m := tc956x_pcie_eth.o
tc956x_pcie_eth-y := tc956xmac_main.o tc956xmac_ethtool.o tc956xmac_mdio.o \
	      mmc_core.o tc956xmac_hwtstamp.o tc956xmac_ptp.o tc956x_xpcs.o tc956x_pma.o \
	      hwif.o  tc956xmac_tc.o dwxgmac2_core.o \
	      dwxgmac2_descs.o dwxgmac2_dma.o tc956x_pci.o \
	      tc956x_msigen.o \
	      tc956x_pcie_logstat.o tc956x_pf_rsc_mng.o \
	      tc956x_qcom.o

ifeq ($(pf), 1)
	tc956x_pcie_eth-y += tc956x_pf_mbx_wrapper.o tc956x_pf_rsc_mng.o tc956x_pf_mbx.o
else ifeq ($(cpe), 1)
EXTRA_CFLAGS+=-DTC956X_CPE_CONFIG
else ifeq ($(port_bridge), 1)
EXTRA_CFLAGS+=-DTC956X_ENABLE_MAC2MAC_BRIDGE
else 
EXTRA_CFLAGS+=-DTC956X_AUTOMOTIVE_CONFIG
endif

ifeq ($(DMA_OFFLOAD), 1)
	tc956x_pcie_eth-y += tc956x_ipa_intf.o
endif	 

ifeq ($(TC956XMAC_SELFTESTS), 1)
	tc956x_pcie_eth-y += tc956xmac_selftests.o
endif
#TC956X_Host_Driver-industrial_limited_tested_20241022_V_04-00-01-QPSSW-210.patch
ccflags-y := -Wmissing-prototypes

endif

modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

modules_install:
	make -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean

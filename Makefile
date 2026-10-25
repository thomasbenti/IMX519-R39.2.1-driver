ccflags-y += -I$(src)/build/nvidia-conftest
ccflags-y += -I$(src)/nvidia-oot/nvidia-oot/include
obj-m += nv_imx519.o

nv_imx519-y := src/nv_imx519.o

KDIR := /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

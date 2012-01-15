all: border-router-eth.elf

CONTIKI=../../..

RF230BB=1
WITH_UIP6=1
UIP_CONF_IPV6=1

#APPS += webserver

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
PROJECT_SOURCEFILES += eth-bridge.c #httpd-simple.c


include $(CONTIKI)/Makefile.include


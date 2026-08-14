.PHONY: liveView test

# deploy-hostname: build and deploy sensor_fusion on SD card
# deploy-%:
# 	NAME=$* rebar3 grisp deploy -n sensor_fusion -v 1.0.0
deploy-nav_1:
	sed -i '/static ip_address=/c\static ip_address=172.20.10.4/28' grisp/grisp2/common/deploy/files/etc/dhcpcd.conf
	NAME=nav_1 rebar3 grisp deploy -n sensor_fusion -v 1.0.0

deploy-nav_2:
	sed -i '/static ip_address=/c\static ip_address=172.20.10.5/28' grisp/grisp2/common/deploy/files/etc/dhcpcd.conf
	NAME=nav_2 rebar3 grisp deploy -n sensor_fusion -v 1.0.0

deploy-nav_3:
	sed -i '/static ip_address=/c\static ip_address=172.20.10.6/28' grisp/grisp2/common/deploy/files/etc/dhcpcd.conf
	NAME=nav_3 rebar3 grisp deploy -n sensor_fusion -v 1.0.0

deploy-uwb_1:
	sed -i '/static ip_address=/c\static ip_address=172.20.10.7/28' grisp/grisp2/common/deploy/files/etc/dhcpcd.conf
	NAME=uwb_1 rebar3 grisp deploy -n sensor_fusion -v 1.0.0

deploy-uwb_2:
	sed -i '/static ip_address=/c\static ip_address=172.20.10.8/28' grisp/grisp2/common/deploy/files/etc/dhcpcd.conf
	NAME=uwb_2 rebar3 grisp deploy -n sensor_fusion -v 1.0.0

deploy-uwb_3:
	sed -i '/static ip_address=/c\static ip_address=172.20.10.9/28' grisp/grisp2/common/deploy/files/etc/dhcpcd.conf
	NAME=uwb_3 rebar3 grisp deploy -n sensor_fusion -v 1.0.0

# screen: show the screen of the grisp connected by usb3 cable
screen:
	sudo screen /dev/ttyUSB1 115200

# remote-hostname: open a remote shell connected to hostname
remote-%:
	erl -sname remote_$* -remsh sensor_fusion@$* -setcookie MyCookie -kernel net_ticktime 8

# shell: open a development shell (not a clean start)
shell:
	rebar3 as computer shell --sname sensor_fusion --setcookie MyCookie

# run_local: start sensor_fusion in release mode (clean start)
run_local:
	./_build/computer/rel/sensor_fusion/bin/sensor_fusion console

# local_release: build sensor_fusion in release mode for the computer
local_release:
	rebar3 as computer release

# start the visualization tool
liveView:
	@cd viewer/ && python3 main.py

# run the unit tests
test:
	rebar3 eunit

# rm_logs: remove files generated when using the shell
rm_logs:
	@rm -f ./measures/*
	@rm -rf ./logs
	@rm -f ./rebar3.crashdump

# clean: remove build files
clean:
	@rm -rf ./_build

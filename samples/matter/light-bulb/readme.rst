.. _matter-light-bulb:

Matter Light bulb sample
########################

Overview
********

Sample for evalaute Matter Dimmable Light bulb endpoint and Matter SDK.
Sample support BLE commisioning to Thread network.

Requirements
************

- Alif Balletto Development Kit
- Matter Thread Border router and Matter Controller
- Pre provisioned Matter Light sample or 3rd oarty light switch

Build Sample
************

Enable automatic start of the openthread stack

.. code-block:: console

   west build -p always -b samples/matter/light-switch

Light-bulb endpoint
*********************

Sample have generated ZAP and Matter file which enable with Matter Root endpoint 0 and OnOff Dimmable Light bulb endpoint 1.

Endpoint 1 OnOff Light switch Server / Client cluster. Client is Toggle light by using Devkit Joytick center button (Press down).
Server cluster is optional but its is  activated for supporting Apple Home ecosystem. Apple will request server for subscribed data model
so Button press will update On/Off light switch server state which is reported to Apple Matter controller. User need to define Automation for Switch event Turn On or Turn Off.

Endpoint 2 Generic Switch for switch server support for supporting sends event notifications `InitialPress` and  `ShortRelease`.
Endpoint use joystick left top corner button. Apple is supporting only single event press event.

User interface
**************

Devkit use 2 leds for indicating Thread and BLE communication state

PWM led (Green Led):
- Dimmable Light controlled by Matter controller or binded On/Off light switch

Blue LED:

- Blinking slowly indicate that Matter BLE Advertisment is active for Matter Provisioning
- Fast Blink (100ms) Means that BLE connection is active and Matter Provisioning is ongoing

Red LED:

- Fast Blinking (100ms) indicate Thread network is active for joining provisionmed newtwork
- 1 Second constant Active Mode indicate That Matter controller subscribed is establis,ent or resumed
- Single short active indicate Endpoint activation

Sample support Balletto Devkit Joystick buttons

Center Butoon `SW0`:

- Hold over 3 seconds for doing Matter factoryreset. After Factory reset press DK normal Reset Button for activate BLE advertisment.

Provisioning information
************************
Scan QR code by Matter Controller device:

`https://project-chip.github.io/connectedhomeip/qrcode.html?data=MT%3A6FCJ142C00KA0648G00`

For `chip-tool` you need to use manual pair code or QR payload.

Manual pair code:
`34970212338`

QR Payload: 
`MT:6FCJ142C000O0648G00`

Evalueate and Provisioning with Apple Home
******************************************

1. Flash builded binary to device and press `Reset` button
2. After reset you shuold see slowly blinking blue led which indicate that Device is advertisment Matter ready device for provision
1. Open `Apple Home` application with Iphone or Ipad which is connected to same Wifi with Matter Conttroler and Thread border router.
2. Click `+` button from app for `Add Accessory`.
3. Scan QR code from `https://project-chip.github.io/connectedhomeip/qrcode.html?data=MT%3A6FCJ142C00KA0648G00`
4. Click `Add To Home` for Start provisioning to Thread neywork and Matter fabric.
5. You shuold see now fast blinking Blue led when BLE connection is established
6. `Aplle Home` will warning Uncertificated Acessory so just click `Add Anyway` for continue
7. When Provision is ready `Apple Home` ask where to add light select a room and click `Continue`
8. Now you can rename a light default is `Matter Accessory` name for example `Matter Light` and click `Continue`
9.  Click `Continue`
10. Click at next window `View at Home` and you can controll now Green led by Moving bar.

Device is now provisioned and Light controll is possible by `Apple Home`. Controlling light by other device user have to add `Automation` at `Apple Home` for connected matter device light switch.
Light on and off need separate `Automation`.
How to create Light On `Automation`:

1. Click `+` button from app for `Add Automation`.
2. Select `An Accessory is Controlled` for use `Matter Switch` controlling a `Matter Light`
3. Select `Matter Switch` from list or your own named light switch device and click `Next`
4. Next page select `Turn On` and click `Next`
5. Select from list `Matter Light` and click `Next`
6. Click `Matter Light` light icon for setting `Turn on` light dimm level and click `Done`

How to create Light Off `Automation`:

1. Click `+` button from app for `Add Automation`.
2. Select `An Accessory is Controlled` for use `Matter Switch` controlling a `Matter Light`
3. Select `Matter Switch` from list or your own named light switch device and click `Next`
4. Next page select `Turn Off` and click `Next`
5. Select from list `Matter Light` and click `Next`
6. Click `Matter Light` light icon for setting `Turn off` light dimm level to 0 click `Done`

Now `Matter Switch` can control `Matter Light` by using light control button.

Evalaute and Provisioning with chip-tool
****************************************

Using `chip-tool` you need to know Thread network active data and Device QR-code payload.

How to get Thread network Active data set:

Open terminal to your linux which is running `Open Thread Border router` and call next command:

.. code-block:: console

    sudo ot-ctl dataset active -x
    35060004001fffe00c0402a0f7f8051000112233445566778899aabbccddee00030e4f70656e54687265616444656d6f0410445f2b5ca6f2a93a55ce570a70efeecb000300001a02081111111122222222010212340708fd110022000000000e0800000003601c0000
    Done


Start `chip-tool` in interactive mode by the following command:



1. Flash builded binary to device and press `Reset` button
2. After reset you shuold see slowly blinking blue led which indicate that Device is advertisment Matter ready device for provision
3. Start `chip-tool` in interactive mode by the following command:

.. code-block:: console

    chip-tool interactive start

4. Provision device by `chip-tool` and assign ``node_id`` 1 and QR code ``MT:6FCJ1-Q0000O0648G00`` by following command

.. code-block:: console

    pairing code-thread 1 hex:35060004001fffe00c0402a0f7f8051000112233445566778899aabbccddee00030e4f70656e54687265616444656d6f0410445f2b5ca6f2a93a55ce570a70efeecb000300001a02081111111122222222010212340708fd110022000000000e0800000003601c0000 MT:6FCJ142C00KA0648G00 --bypass-attestation-verifier true

Flash binary to device first. After flash and reset you shuold see slowly blinking blue led which indicate that Device is advertisment Matter ready
device for provision. Provisioning device to Matter network by your using Matter ecosystem. At provisioning phase Blue led start blinking faster when device is provisioned to Thread and Matter Fabric.
After Thread network provisioning blue led is disabled and Red led start blinking when Matter controller is open Secure connection to device and possible request Subscribe event for On/Off Dimmable Light.

Configure Light-bulb
************************


Using `chip-tool` device need to bind with provisioned light-switch endpoint. User need to know Light light-switch `node-id` which is given at provisioned phase.
Update Light bulb's access list to include the light switch by the following command for `node-id` 1:

.. code-block:: console

    accesscontrol write acl '[{"fabricIndex": 1, "privilege": 5, "authMode": 2, "subjects": [112233], "targets": null}, {"fabricIndex": 1, "privilege": 3, "authMode": 2, "subjects": [2], "targets": [{"cluster": 6, "endpoint": 1, "deviceType": null}, {"cluster": 8, "endpoint": 1, "deviceType": null}]}]' 1 0

Now Bind On/Off switch to Light bulb `node-id` 1 by:

.. code-block:: console

    binding write binding '[{"fabricIndex": 1, "node": 1, "endpoint": 1, "cluster": 6}, {"fabricIndex": 1, "node": 1, "endpoint": 1, "cluster": 8}]' 2 1

Now the light switch device can control the light bulb device's led.


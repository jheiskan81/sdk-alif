.. _matter-light-switch:

Matter Light switch sample
##########################

Overview
********

Sample for evalaute Matter Light switch endpoint and Matter SDK.
Sample support BLE commisioning to Thread network.

Requirements
************

- Alif Balletto Development Kit
- Matter Thread Border router and Matter Controller
- Preprovisioned Matter Light Bulb device

Build Sample
************

Enable automatic start of the openthread stack

.. code-block:: console

   west build -p always -b samples/matter/light-switch

Light-switch endpoint
*********************

Sample have generated ZAP and Matter file which enable with Matter Root endpoint 0 and OnOff Light Switch and Generic Switch endpoints.

Endpoint 1 OnOff Light switch Server / Client cluster. Client is Toggle light by using Devkit Joytick center button (Press down).
Server cluster is optional but its is  activated for supporting Apple Home ecosystem. Apple will request server for subscribed data model
so Button press will update On/Off light switch server state which is reported to Apple Matter controller. User need to define Automation for Switch event Turn On or Turn Off.

Endpoint 2 Generic Switch for switch server support for supporting sends event notifications `InitialPress` and  `ShortRelease`.
Endpoint use joystick left top corner button. Apple is supporting only single event press event.

User interface
**************

Devkit use 2 leds for indicating Thread and BLE communication state

Blue LED:

- Blinking slowly indicate that Matter BLE Advertisment is active for Matter Provisioning
- Fast Blink (100ms) Means that BLE connection is active and Matter Provisioning is ongoing

Red LED:

- Fast Blinking (100ms) indicate Thread network is active for joining provisionmed newtwork
- 1 Second constant Active Mode indicate That Matter controller subscribed is establis,ent or resumed
- Single short active indicate Endpoint activation

Sample support Balletto Devkit Joystick buttons

Center Butoon `SW0`:

- Press down will toggle OnOff-switch endpoint state

Left Up Corner Button `SW1`:

- Generic switch for generate `InitialPress` at active state  and  `ShortRelease` after release button.

Right Up Corner Button `SW2`:

- Hold over 3 seconds for doing Matter factoryreset. After Factory reset press DK normal Reset Button for activate BLE advertisment.

Provisioning information
************************
Scan QR code by Matter Controller device:

https://project-chip.github.io/connectedhomeip/qrcode.html?data=MT%3A4CT9142C00KA0648G00

For `chip-tool` you need to use manual pair code or QR payload.

Manual pair code:
`34970112332`

QR Payload: 
`MT:4CT91AFN00KA0648G00`

Evalaute
********

Flash binary to device first. After flash and reset you shuold see slowly blinking blue led which indicate that Device is advertisment Matter ready
device for provision. Provisioning device to Matter network by your using Matter ecosystem. At provisioning phase Blue led start blinking faster when device is provisioned to Thread andMatter Fabric.
After Thread network provisioning blue led is disabled and Red led start blinking when Matter controller is open Secure connection to device and possible request Subscribe event for Generic switch and OnOff Light-switch.

Configure Light-switch's
************************

With  `Apple Home` system device will report automatically if user Press `SW0` On/Off Light-switch  or `SW1` Generic switch. User have to add `Automation` at `Apple Home` for connected matter light.

Using `chip-tool` device need to bind to provisioned light-bulb endpoint. User need to know Light bulb `node-id` which is given at provisioned phase.
On/Off Light switch bind to Light bulb needs first Acced control list permission for On/Off Light cluster control.
Update Light bulb's access list to include the light switch by the following command for `node-id` 1:

.. code-block:: console

    accesscontrol write acl '[{"fabricIndex": 1, "privilege": 5, "authMode": 2, "subjects": [112233], "targets": null}, {"fabricIndex": 1, "privilege": 3, "authMode": 2, "subjects": [2], "targets": [{"cluster": 6, "endpoint": 1, "deviceType": null}, {"cluster": 8, "endpoint": 1, "deviceType": null}]}]' 1 0

Now Bind On/Off switch to Light bulb `node-id` 1 by:

.. code-block:: console

    binding write binding '[{"fabricIndex": 1, "node": 1, "endpoint": 1, "cluster": 6}, {"fabricIndex": 1, "node": 1, "endpoint": 1, "cluster": 8}]' 2 1

Now the light switch device can control the light bulb device's led.


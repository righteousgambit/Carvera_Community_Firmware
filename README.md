# **Overview**

This is a community branch of the Smoothieware firmware for the Makera Carvera
CNC Machine. It is also part of the Carvera Community Org, which includes these
other projects: [Machine Profiles](https://github.com/Carvera-Community/Carvera_Community_Profiles) -
Post processors, 3d design files, and tool libraries for various CAM/CAD
programs

Open Source Version of the Carvera
[Controller](https://github.com/Carvera-Community/CarveraController) - includes
builds for linux and macOS. There is a javascript based controller alternative
in the works as well

[Trello](https://trello.com/b/qKxPlEbk/carvera-community-firmware-controller-and-other-tech)
- for seeing progress on features and making recommendations

**NOTE** it is not necessary to build the firmware yourself unless you want to.
prebuilt binaries are available
[here](https://github.com/Carvera-Community/Carvera_Community_Firmware/releases).
There will be periodic stable releases that line up with controller releases,
and rolling beta versions to test new features.

Smoothie is a free, opensource, high performance G-code interpreter and CNC
controller written in Object-Oriented C++ for the LPC17xx micro-controller ( ARM
Cortex M3 architecture ). It will run on a mBed, a LPCXpresso, a SmoothieBoard,
R2C2 or any other LPC17xx-based board. The motion control part is a port of the
awesome grbl.

# Documentation and resources

- [Community feeds, speeds and
  accessories](https://docs.google.com/spreadsheets/d/1i9jD0Tg6wzTpGYVqhLZMyLFN7pMlSfdfgpQIxDKbojc/edit) Comprensive
  resource for G/M codes, config variables, console commands, as well as feeds & speeds recommendations.  
- [Makera website](https://wiki.makera.com/en/home), [supported codes](https://wiki.makera.com/en/supported-codes), [feeds & speeds](https://wiki.makera.com/en/speeds-and-feeds)
- [Smoothieware documentation](https://smoothieware.github.io/Webif-pack/documentation/web/html/index.html)
- [Carvera A to Z](https://carvera-a-to-z.gitbook.io/carvera-a-to-z) - A work in
  progress wiki for all sorts of information on getting started with the Carvera
  CNC machine

_**More from the Carvera Community**_

- [Carvera Controller](https://github.com/carvera-community/carvera_controller/)
  - community controller with extensive additional features and support for
  community firmware features.
- [Carvera Community Profiles](https://github.com/Carvera-Community/Carvera_Community_Profiles) - profiles and post-processor for various third party CAM software.
- [Carvera CLI](https://github.com/hagmonk/carvera-cli) - CLI interface to
  Carvera for scripting and device management.

_**Other open source tools**_

- https://cc.grid.space/ 
- https://github.com/GridSpace/carve-control
- https://github.com/AngryApostrophe/Clout
- https://cnc.js.org/ 
- https://github.com/cncjs/cncjs-pendant-boilerplate

Work in progress wireless 3 axis touch probe: will be released open source and
open hardware along with a purchasable version
https://github.com/faecorrigan/Open-Source-3-axis-CNC-Touch-Probe

# Filing issues and contributing 

Please follow [the Smoothieware issue template](https://github.com/Smoothieware/Smoothieware/blob/edge/ISSUE_TEMPLATE.md)
when filing bugs against this repo.

Contributions very welcome! 

- Open an
[issue](https://github.com/Carvera-Community/Carvera_Community_Firmware/issues)
either on github, trello, or message one of the admins. Issues can be for
bugfixes or feature requests. 
- Test beta versions of the firmware and give bugreports/feedback
- Contribute pull requests to the project
- Contribute to the [A_To_Z wiki](https://github.com/SergeBakharev/carvera_a_to_z)

Carvera Community Firmware uses the same guidelines as upstream Smoothieware
- http://smoothieware.org/coding-standards
- http://smoothieware.org/developers-guide
- http://smoothieware.org/contribution-guidlines

When you are working on a pull request, please adhere to the following requirements:

- Pull requests should be limited to one feature, or one bugfix/issue,
  or small set of highly related issues that have real world impact on the machines.
- Pull requests should be standalone and based on the `Dev` branch unless there is
  a very good reason for them to depend on each other.
- Issues and PR bodies are for human to human communication. If using AI assistance,
  you are responsible for reviewing the content and not submitting something that
  you wouldn't write yourself.
- The expectation for all PRs is that the submitter has done everything reasonable
  to review and test the PR on their side. This includes running the code against
  your own machine hardware.
- We allow AI assisted pull requests in this repository. Please use the AGENTS.md
  file if you are using an AI agent, and fill out the relevant information in the
  pull request template.
- Do not scan the code base with the express purpose of bulk creating issues/PRs
  without prior approval from maintainers.


# Developers

See [DEVELOPER.md](./DEVELOPER.md) for instructions on building the firmware and
debugging.

See also [TODO](./TODO.md) for some ideas on things to work on :)

# Donate

This particular branch of the carvera firmware is maintained by [Fae
Corrigan](https://www.patreon.com/propsmonster) For smoothieware as a whole: the
Smoothie firmware is free software developed by volunteers. If you find this
software useful, want to say thanks and encourage development, please consider a
[Donation](https://paypal.me/smoothieware)

# License

- Smoothieware is released under the GNU GPL v3, which you can find at
http://www.gnu.org/licenses/gpl-3.0.en.html 
- MRI is released under Apache 2.0, which you can find at
https://www.apache.org/licenses/LICENSE-2.0


## Sensor records

Status responses include optional numeric `SE`, `SI`, `SC`, `SS`, `SA`, `SF`,
`SP`, `ST`, and `SQ` fields. They describe sample identity, availability, configured
input states, executing versus parsed lines, spindle statistics, ATC stages,
and fault persistence. Position is derived from issued steps; feed is a planner
value. Temperature ages refer to computed sensor readings, and probe-voltage
age refers to receipt over the probe UART.

`M964` reads the bounded event history while idle or halted. `M964 L1` reads the
last validated SD fault record with its original boot identity. `M964 R1` reads
and rearms history while idle and unhalted. `EH` starts a record, `EV` carries an
event, and `EE:0` confirms completion; `EE:1` means unavailable or incomplete.
Qualified probe/setter contacts emit `PRH` records from a two-contact main-loop
queue. `SQ` reports contact sequence, cumulative dropped contacts, pending
contacts, fault-write duration, and the last rearm result (0 not requested,
1 accepted, 2 rejected). Sequence numbers wrap modulo 2^32, including zero.
Dropped-contact and overwritten-event counts saturate instead of wrapping.

`abort -R <nonce>` accepts a nonzero decimal 32-bit nonce. After disposing of
buffered commands and macro returns, it replies `{RB:1,<nonce>}` to the requesting
stream when abort handling returns, including when no file is playing. Queue
disposal happens at entry; the receipt is withheld during abort's motion waits.
Physical stop still requires a fresh idle status. Other abort parameters keep
their existing behavior and receive no correlated receipt.

The boot counter uses `/sd/sensor-boot.dat`, with two copies of each reserved
identity before it is issued. A single corrupt legacy counter slot leaves
identity unavailable because the lost slot could have held a newer identity.
Fault records alternate between
`/sd/sensor-fault0.dat` and `/sd/sensor-fault1.dat`. Boot identity is scoped to
that SD state; failed writes or corrupt counters leave identity unavailable.
Fault persistence runs once after a halt and drained motion in the main loop,
so loss of power before
that write can still lose the current fault. The payload is flushed before
its commit header and the complete record is validated after close. These
checks cannot guarantee SD-card durability during physical power loss. An
incomplete or corrupt record leaves the last validated alternate slot intact.
The fixed 16-event history spans
about 1.6 seconds when only the 10 Hz periodic records occur; transitions shorten
that span.

`spindle.stall_detection_enable` defaults to `false`. Enabling it uses the
configured spin-up delay and stall thresholds. Machine qualification is required
before relying on this check. Portable state/storage checks are in
`tests/TEST_sensor_evidence/test_state.cpp` and can be compiled with C++11 and
`-Isrc/libs`.

G-code parenthesis comments preserve executable words after the closing
parenthesis. Nested or unmatched parentheses in executable G-code halt before
the block is dispatched. Text and filename commands preserve their payload
without interpreting its punctuation or G-code words. Integer `G90`/`G91` words apply before motion on the same block;
decimal `G90.x`/`G91.x` modes are unsupported and halt before dispatch.

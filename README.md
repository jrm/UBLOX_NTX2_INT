UBLOX_NTX2
==============

High altitude balloon telemetry.

An Arduino sketch driving a UBLOX GPS and NTX2 433 Mhz NFM Transmitter.

Uses an ISR for transmission of RTTY telemetry strings.

Todo:

 * Enable pauses between strings, possibly variable pauses  - akin to APRS smart beaconing.
 * Enable radio power-down during pauses to conserve energy.
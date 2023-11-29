# Can Bus Dump

<div style="text-align: justify">
The individual Can messages on the bus from heat pumps and FEK direction can be found in the CSV files. The Python file provides a starting point for decoding the messages.
</div>

## Message Decoding
- Each CAN message is made up of 7 bytes.

- The can id, which describes the device sending the message, is specified by Stiebel Eltron and is made up of the combination of the first 4 bits of the first byte and the last 4 bits of the second byte.
```
can_id = 8 * (byte_data[0] & 0b11110000) + (byte_data[1] & 0b00001111)
```

- The last 4 bits of the first byte describe the type of can message. Value 1 describes a request, value 2 a response. Value 0 is a special case and points to byte two. If the second byte value is 0x79, the message sent is a broadcast message that is sent to all bus members at regular intervals.
```
message_type = 8 * (byte_data[0] & 0b00001111)
```

- The third byte is used for extention frame messages, is not currently used and is labeled 0xfa.

- Bytes four and five together form the value id which is sent or received by the bus members.
```
message_id = (byte_data[4] << 8) | byte_data[5]
```

- Bytes six and seven together form the value of the message which is sent or received by the bus participants. In the case of a request message, the bytes are written with the value zero.
```
message_value = (byte_data[6] << 8) | byte_data[7]
```

### Special Messages from FEK/FET
<div style="text-align: justify">
It is interesting to note that some values are not transmitted via the way mentioned above. The FEK/FET sends a CAN message, with the data ID coded via the second byte. The message value is coded as Int16 via bytes 3 and 4. Bytes 5 and 6 are written with the value zero.
</div>

### Examples

TODO

## Note
<div style="text-align: justify">
It should be mentioned that an ISGWeb can also be found on the heat pump page. The heat pump was in normal heating mode at the time of recording.
</div>
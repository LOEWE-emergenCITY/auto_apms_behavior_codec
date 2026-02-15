# AutoAPMS Behavior Codec

Efficient de-/serialization of AutoAPMS behaviors for robust over-the-air mission updates.

## Message Fromat
The current approach for message encoding is the following:
### Document
The document object is encoded as a CBOR array with two elements, first a string encoding the "main_tree_to_execute" and second a CBOR array containg the behavior trees. Potentital room for optimization is to skip the "main_tree_to_Execute" and just have it bee the first element of the tree array, in this case the document would just be an array of behavior trees.

### Behavior Tree
A behavior tree is encoded as a CBOR array with 2 elements, first the name of the tree and second the root node. Potetnial room for improvement could be using as short as possible tree names.

### Node
A node is encoded as a CBOR array with 3 elements, first the type code of the node, second an array of the node's ports and third an array off child nodes. All arrays may also have size 0. Potential room for imrpovement could be omitting empty arrays completely.

### Ports
Each port consists of an array, the elements of which depend on the type of port. Currently all ports conaint the port id (just a mapping of the ports position in the node) this can be ommited if it is known that the ports are always ordered in a specific way.

# Information about LoRa throughput
The following is intended as a reference to judge possible lora throughput

The following assumes the following LoRa parameters:

    SF7, CR5, BW 125, Preamble Length 8, CRC 2 byte

This calculator: https://iftnt.github.io/lora-air-time/index.html has been used for most calculations.

Static airtime per transmissions due to LoRa preamble, header, and CRC: `25.86ms`
Per Byte an additional airtime (approximation): `1.47ms` (for large payloads) (calculated from the difference ein payload airtime between a 1 and a 255 byte payload)
Allowed airtime per hour with 1% duty cycle: `36000ms`
Resulting from this and the overhead mentioned in [LoRa Protocol](#lora-protocol), the following maximal throughput could be possible:
|Payload Size (byte)    | Airtime per message (ms)  | maximum messages per hour | Min. message spacing (s)  |effective data rate (byte/s)|
|----                   | ---                       | ---                       | ---                       |---                         |
|1                      |34,68                      |1038                       |3,47                       |0,29
|2                      |36,15                      |995                        |3,62                       |0,55
|4                      |39,09                      |920                        |3,91                       |1,02
|8                      |44,97                      |800                        |4,5                        |1,78
|16                     |56,73                      |634                        |5,68                       |2,82
|32                     |80,25                      |448                        |8,04                       |3,98
|64                     |127,29                     |282                        |12,77                      |5,01
|128                    |221,37                     |162                        |22,22                      |5,76
|250                    |400,71                     |89                         |40,45                      |6,18

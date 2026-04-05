# AutoAPMS Behavior Codec

Efficient de-/serialization of AutoAPMS behaviors for robust over-the-air mission updates.

## Dictionary
For encoding the behavior trees a dictionary is build from all available node maniftests. This is done by the [DictionaryManager](auto_apms_behavior_codec/include/auto_apms_behavior_codec/dictionary_manager.hpp). It creates a mapping from node names to numeric IDs.
It also offers a function to get a merged manifest of all manifests which were used to build the dictionary, or a Vector of the individual manifests. This is used to register the node types with the `TreeDocument` during encoding and decoding.

>[!WARNING] 
>It is very important that the dictionary used for decoding and encoding is equivalent. To ensure this, make sure that the same node manifests are registered in the Workspace. Currently there is no mechanism to check decodeing correctness, this should be added in the future.

## Execution Flow
### Encoding
The encoding process is as follows:
1. The XML representation of the behavior tree is received from the input topic and read into an `auto_apms_behavior_tree::core::TreeDocument` using the Api provided by the `auto_apms_behavior_tree` package. The node manifest returned by the `DictionaryManager` is registered with this `TreeDocument`.
2. The `TreeDocument` is then tranfered into an internal represetnation for easier encoding, see (/auto_apms_behavior_codec/include/auto_apms_behavior_codec/internal_representation.hpp).
3. The internal representation is then encoded into a byte array using CBOR. And send to the output topic.

### Decoding
The decoding process is the opposite of the encoding process:
1. The encoded byte array is received from the input topic and decoded into the internal representation.
2. The internal representation is then transformed into a `TreeDocument` object, again using the Api provided by the `auto_apms_behavior_tree` package. The node manifest returned by the `DictionaryManager` is registered with this `TreeDocument`.
3. The `TreeDocument` is then converted back into an XML representation and published on the output topic.

## Encoding Format
The current approach for message encoding is the following:
### Document
The document object is encoded as a CBOR array with an element per tree and a boolean as its first element. The first contained tree is the main tree to execute, except if the boolean is true, in this case no "main_tree_to_execute" is specified.

### Behavior Tree
A behavior tree is encoded as a CBOR array with 2 elements, first the name of the tree and second the root node. Potential room for improvement could be using as short as possible tree names.

### Node
A node is encoded as a CBOR array with 3 elements, first the type code of the node, second an array of the node's ports and third an array off child nodes. All arrays may also have size 0. Empty arrays of children are omitted.

### Ports
Each port consists of an array, the elements of which depend on the type of port. Currently all ports contain the port id (just a mapping of the ports position in the node) this can be omitted if it is known that the ports are always ordered in a specific way.
A special "invalid" port encoding is defined, this does not have an unsiged integer as a first element, but a string with the value originally conatined in the XML file. This is a workauround for remapping.

#### Special Handling of SubTree Ports
The ports of a SubTree are not known beforehand, therefore the names of the ports are included in the encoded version and the value as string. Additionally the "_autoremap" field is included as bool.

### Current State
Currently the encoding is functional, exept for handling of SubTrees. The [example document](auto_apms_behavior_codec_examples/behavior/hello_world.xml) is encoded to: 
`82 82 61 4d 83 18 19 80 82 82 18 1e 83 83 00 62 49 44 61 50 83 01 63 6d 73 67 65 48 65 6c 6c 6f 02 82 18 1e 83 83 00 62 49 44 61 50 83 01 63 6d 73 67 65 57 6f 72 6c 64 02 82 61 50 82 18 36 82 82 00 65 7b 6d 73 67 7d 82 01 64 49 4e 46 4f`

Using a CBOR analysis tool, such as https://cbor.me/, the structure described above is nicley visible.

## ROS nodes
The package contains a ROS node for encoding and two decoder variants. The encoder subscribes to a topic with the XML representation of the behavior tree and publishes the encoded version on another topic.

### Encoder
The `tree_encoder_subscriber` subscribes to XML behavior tree messages and publishes the CBOR-encoded version.

```bash
ros2 run auto_apms_behavior_codec tree_encoder_subscriber --ros-args --params-file install/auto_apms_behavior_codec_examples/share/auto_apms_behavior_codec_examples/config/goto_codec_params.yaml
```

### Decoder Publisher
The `tree_decoder_publisher` subscribes to the encoded version and publishes the reconstructed XML representation on a topic.

```bash
ros2 run auto_apms_behavior_codec tree_decoder_publisher --ros-args --params-file install/auto_apms_behavior_codec_examples/share/auto_apms_behavior_codec_examples/config/goto_codec_params.yaml
```

### Decoder Executor Client
The `tree_decoder_executor_client` subscribes to encoded behavior trees and, upon decoding, sends the resulting XML to AutoAPMS's `TreeExecutorNode` via the `StartTreeExecutor` action interface. Each received message cancels any currently running execution (waiting for termination) before starting the new tree.

**Parameters** (in addition to the common decoder base parameters `node_manifest` and `encoded_in_topic`):
| Parameter | Default | Description |
|---|---|---|
| `start_tree_executor_action_name` | `tree_executor/start` | Action name for the `StartTreeExecutor` action server |

The action goal is populated with:
- `build_request`: the decoded XML string
- `node_manifest`: the encoded YAML of the node manifest used by the decoder's dictionary
- `build_handler`: `auto_apms_behavior_tree::TreeFromStringBuildHandler`

```bash
ros2 run auto_apms_behavior_codec tree_decoder_executor_client --ros-args --params-file install/auto_apms_behavior_codec_examples/share/auto_apms_behavior_codec_examples/config/goto_codec_params.yaml
```

### Sending data to the encoder

```bash
ros2 topic pub --once /xml_in auto_apms_behavior_codec_interfaces/msg/TreeXmlMessage "{tree_xml_message: 'XML_HERE'}"
```

From a file this could look like (make sure to have `auto_apms_ros2behavior` installed):

```bash
TREE_IDENTITY=auto_apms_behavior_codec_examples::hello_world::Main
ros2 topic pub --once /xml_in auto_apms_behavior_codec_interfaces/msg/TreeXmlMessage "{tree_xml_message: '$(ros2 behavior show $TREE_IDENTITY)'}"
```


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

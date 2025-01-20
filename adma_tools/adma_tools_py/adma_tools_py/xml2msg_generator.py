import rclpy
import rclpy.logging
from rclpy.node import Node
import xml.etree.ElementTree as ET
import json
from ament_index_python.packages import get_package_share_directory
import os

class XML2MSGGenerator(Node):
        def __init__(self):
                super().__init__("xml2msg_generator")
                self.xml_file = self.declare_parameter('xml_file', '$HOME/ros2_ws/loki/src/adma_ros_driver/adma_tools/adma_tools_py/config/ADMA_UDP-DataStream_DELTA11_v7.0_v30.5.1.0.xml').value
                self.msg_output_file = self.declare_parameter('msg_output_file', '$HOME/ros2_ws/src/adma_tools/adma_tools_py/config/generated.msg').value
                
                # load json glossar for datatype remapping
                self.glossar = None
                package_share_directory = get_package_share_directory('adma_tools_py')
                glossar_path = os.path.join(package_share_directory, 'config', 'datatypes_glossar.json')
                # print(glossar_path)
                with open(glossar_path, 'r') as file:
                        self.glossar = json.load(file)

                self.reserved_block_counter = 0

                msg_content = self.parse_xml_to_ros_msg(self.xml_file)
                self.save_msg_file(msg_content, self.msg_output_file)
                rclpy.shutdown()
        
        def parse_xml_to_ros_msg(self, xml_file):
                # load and parse xml file
                tree = ET.parse(xml_file)
                root = tree.getroot()
                msg_content = []

                # extract message name from header
                msg_name = root.find('FormatHeader').find('FormatName').text
                msg_name += root.find('FormatHeader').find('FormatVersion').text
                # add "reserved" block for the ABD Header
                msg_content.append("byte[8] abd_header") 
                print('Generating msg: ' + str(msg_name))

                # load data packets from xml
                datapackets = root.find('MeasurementData').findall('Package')

                #TODO: extract delta logic to reuse this for ADMANet data
                delta_datatypes = self.glossar['delta']
                
                for package in datapackets:
                        channels = package.findall('Channel')
                        for channel in channels:
                                channel_name = channel.get('Name').lower()
                                channel_datatype = channel.find('DataType').text
                                channel_datatype = self.find_datatype(delta_datatypes, channel_datatype)
                                channel_byte_offset = channel.find('ByteOffset') # may useful for validation
                                if channel_name == 'code_version':
                                        msg_content.append(self.add_reserved_block(2))
                                        msg_content.append(f"{channel_datatype} {channel_name}") 
                                        # msg_content.append("byte[4] reserved_2") #TODO: invalid docs!
                                elif channel_name == 'angle_of_orientation':
                                        msg_content.append(f"{channel_datatype} {channel_name}") 
                                        msg_content.append(self.add_reserved_block(4))
                                elif channel_name == 'delta_time':
                                        #TODO: validate XML!
                                        channel_datatype = 'int32'
                                        msg_content.append(f"{channel_datatype} {channel_name}") 
                                else:
                                        msg_content.append(f"{channel_datatype} {channel_name}") 

                msg_str = f"# Generated from XML\n"
                # msg_str += f"{msg_name} msg\n\n"
                for line in msg_content:
                        msg_str += line + "\n"

                return msg_str

        def save_msg_file(self, msg_str, output_file):
                with open(output_file, 'w') as file:
                        file.write(msg_str)

        """
        Function to find the correct ROS datatype correspondending to the ADMA datatypes

        Parameters:
        datatypes: known datatypes loaded from the glossar.json
        adma_datatype: current ADMA datatype loaded from XML

        Returns:
        string datatype
        """
        def find_datatype(self, data_types, adma_datatype):
                for datatype in data_types:
                      if datatype['adma_datatype'] == adma_datatype:
                             return datatype['ros_datatype']
                self.get_logger().warn(f"Couldnt find datatype: {adma_datatype}")
        
        """
        Function to add a reserved block in the ROS msg to ensure the data is ordered correctly

        Parameters:
        datasize (int): size of the reserved block

        Returns:
        string text to append to ROS msg
        """
        def add_reserved_block(self, datasize):
               reserved_block = f"byte[{datasize}] reserved_{self.reserved_block_counter}"
               self.reserved_block_counter += 1
               return reserved_block
               

def main(args=None):
    rclpy.init(args=args)
    generator = XML2MSGGenerator()
    while(rclpy.ok()):
          rclpy.spin(generator)
    

if __name__ == "__main__":
    main()

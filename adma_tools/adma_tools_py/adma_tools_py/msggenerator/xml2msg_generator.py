import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET
import abc


class XML2MSGGenerator(Node, abc.ABC):
    def __init__(self):
        super().__init__("xml2msg_generator")
        self.xml_file = self.declare_parameter(
            'xml_file',
            '$HOME/ros2_ws/loki/src/adma_ros_driver/adma_tools/adma_tools_py/config/ADMA_UDP-DataStream_DELTA11_v7.0_v30.5.1.0.xml',
        ).value
        self.msg_output_file = self.declare_parameter(
            'msg_output_file', '$HOME/ros2_ws/src/adma_tools/adma_tools_py/config/generated.msg'
        ).value

        # load json glossar for datatype remapping
        self.glossar = None
        self.load_glossar()

        # some flags for usability
        self.reserved_block_counter = 0
        self.msg_content = []
        self.xml_root_tree = None
        self.load_xml_file()

        self.parse_xml_to_ros_msg()
        msg_text = self.convert_content_to_msg()
        self.save_msg_file(msg_text, self.msg_output_file)
        rclpy.shutdown()

    @abc.abstractmethod
    def load_glossar(self):
        raise NotImplementedError

    def load_xml_file(self):
        tree = ET.parse(self.xml_file)
        self.xml_root_tree = tree.getroot()
        # extract message name from header
        msg_name = self.xml_root_tree.find('FormatHeader').find('FormatName').text
        msg_name += self.xml_root_tree.find('FormatHeader').find('FormatVersion').text
        # print('Generating msg: ' + str(msg_name))
        self.msg_content.append(
            f"# {msg_name} generated from XML by adma_tools_py from the offical ADMA ROS Driver\n"
        )
        msg_name = msg_name.replace('.', '')
        # TODO: may can be used for file name?!

    @abc.abstractmethod
    def add_content_to_msg(self, channel_datatype, channel_name):
        raise NotImplementedError

    def parse_xml_to_ros_msg(self):
        # load data packets from xml
        datapackets = self.xml_root_tree.find('MeasurementData').findall('Package')

        for package in datapackets:
            channels = package.findall('Channel')
            for channel in channels:
                channel_name = channel.get('Name').lower()
                channel_datatype = channel.find('DataType').text
                channel_datatype = self.find_datatype(channel_datatype)
                channel_byte_offset = channel.find('ByteOffset')  # may useful for validation
                self.add_content_to_msg(channel_datatype, channel_name)

    def convert_content_to_msg(self):
        msg_str = ''
        for line in self.msg_content:
            msg_str += line + '\n'

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

    def find_datatype(self, adma_datatype):
        for datatype in self.glossar:
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

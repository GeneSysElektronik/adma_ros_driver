import json
import os

from adma_tools_py.msggenerator.xml2msg_generator import XML2MSGGenerator
from ament_index_python.packages import get_package_share_directory
import rclpy


class AddonDeltaGenerator(XML2MSGGenerator):

    def __init__(self):
        super().__init__()

    def load_glossar(self):
        package_share_directory = get_package_share_directory('adma_tools_py')
        glossar_path = os.path.join(package_share_directory, 'config', 'datatypes_glossar.json')
        glossar_content = None
        with open(glossar_path, 'r') as file:
            glossar_content = json.load(file)
        self.glossar = glossar_content['delta']

    def load_xml_file(self):
        super().load_xml_file()
        # add 'reserved' block for the ABD Header
        self.msg_content.append('byte[8] abd_header')

    def add_content_to_msg(self, channel_datatype, channel_name):
        if channel_name == 'code_version':
            self.msg_content.append(self.add_reserved_block(2))
            self.msg_content.append(f'{channel_datatype} {channel_name}')
        elif channel_name == 'angle_of_orientation':
            self.msg_content.append(f'{channel_datatype} {channel_name}')
            self.msg_content.append(self.add_reserved_block(4))
        elif channel_name == 'delta_time':
            channel_datatype = 'int32'
            self.msg_content.append(f'{channel_datatype} {channel_name}')
        else:
            self.msg_content.append(f'{channel_datatype} {channel_name}')


def main(args=None):
    rclpy.init(args=args)
    generator = AddonDeltaGenerator()
    while rclpy.ok():
        rclpy.spin(generator)


if __name__ == '__main__':
    main()

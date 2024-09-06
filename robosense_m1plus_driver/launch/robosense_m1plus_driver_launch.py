# Robosense M1 Plus Driver

# Alexandru Cretu <alexandru.cretu@alumni.uniroma2.eu>
# Intelligent Systems Lab <isl.torvergata@gmail.com>

# 6 September, 2024

# This is free software.
# You can redistribute it and/or modify this file under the
# terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.

# This file is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE. See the GNU General Public License for more details.

# You should have received a copy of the GNU General Public License along with
# this file; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robosense_m1plus_driver',
            executable='robosense_m1plus_driver',
            name='robosense_m1plus_driver',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('robosense_m1plus_driver'),'config','config.yaml')]
        )
    ])

# Copyright 2020 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import subprocess

cmd = 'ros2 run performance_test perf_test --communication ROS2  \
 --max-runtime 10 --disable-logging '

db_args = ''  # set the db parameters to upload the result to the database


# launch 3 sets of Array1m pub/sub, 1 to 1
for i in range(0, 3):
    args = ' --rate 50 --msg Array1m --topic Array1m_' + str(i) + ' -l log_pub -p 1 -s 0 '
    cc = cmd + args + db_args
    p = subprocess.Popen(cc, shell=True)

for i in range(0, 3):
    args = ' --rate 50 --msg Array1m --topic Array1m_' + str(i) + ' -l log_sub -p 0 -s 1 '
    cc = cmd + args + db_args
    p = subprocess.Popen(cc, shell=True)


# launch 4 sets of PointCloud4m pub/sub, 1 to 1
for i in range(0, 4):
    args = ' --rate 20 --msg PointCloud4m --topic PointCloud4m_' + str(i) + \
        ' -l log_pub -p 1 -s 0 '
    cc = cmd + args + db_args
    p = subprocess.Popen(cc, shell=True)

for i in range(0, 4):
    args = ' --rate 20 --msg PointCloud4m --topic PointCloud4m_' + str(i) + \
        ' -l log_sub -p 0 -s 1 '
    cc = cmd + args + db_args
    p = subprocess.Popen(cc, shell=True)


# launch 10 sets of Array1k, 1 pub to 5 subs
for i in range(0, 10):
    args = ' --rate 100  --msg Array1k --topic Array1k_' + str(i) + ' -l log_pub -p 1 -s 0 '
    cc = cmd + args + db_args
    p = subprocess.Popen(cc, shell=True)

for i in range(0, 10):
    for j in range(0, 5):
        args = ' --rate 100  --msg Array1k --topic Array1k_' + str(i) + ' -l log_sub_' + \
            str(j) + ' -p 0 -s 1 '
        cc = cmd + args + db_args
        p = subprocess.Popen(cc, shell=True)

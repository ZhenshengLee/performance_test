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

import itertools

import pandas


def load_logfile(filename):
    """Load logfile into header dictionary and pandas dataframe."""
    with open(filename) as source:
        header = {}
        for item in itertools.takewhile(lambda x: not x.startswith('---'), source):
            if not item.strip():  # Don't care about whitespace-only lines
                continue
            try:
                key = item.split(':')[0].strip()
                value = item.split(':', maxsplit=1)[1].strip()
                header[key] = value
            except Exception:
                print('Error trying to parse header line "{}"'.format(item))
                raise
        dataframe = pandas.read_csv(source, sep='[ \t]*,[ \t]*', engine='python')
        unnamed = [col for col in dataframe.keys() if col.startswith('Unnamed: ')]
        if unnamed:
            dataframe.drop(unnamed, axis=1, inplace=True)
    return header, dataframe


def load_logfiles(logfiles):
    """Load logfiles into header dictionaries and pandas dataframes."""
    headers = []
    dataframes = []

    for logfile in logfiles.value:
        header, dataframe = load_logfile(logfile)
        headers.append(header)
        dataframes.append(dataframe)

    return headers, dataframes

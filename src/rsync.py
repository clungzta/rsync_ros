#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Alex McClung
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the author may be used to endorse or promote 
#    products derived from this software without specific prior
#    written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import re
from subprocess import Popen, PIPE

class Rsync():

    def __init__(self, rsync_args, source, dest, progress_callback=None):
        self.rsync_args = rsync_args
        self.source = source
        self.dest = dest
        self.percent_complete = 0
        self.progress_callback = progress_callback
        self.transfer_rate = 0

        self.stdout_block = ''
        self.stderr_block = ''
    
    def sync(self):
        #Sync the files
        self.rsync_cmd = ['rsync'] + self.rsync_args + ['--progress', '--outbuf=L', self.source, self.dest]
        self.p = Popen(self.rsync_cmd, stdout=PIPE, stderr=PIPE)
        
        #Catch stdout from RSync in (near) real-time
        for line_with_whitespace in iter(self.p.stdout.readline, b''):
            self.line = re.sub( '\s+', ' ', line_with_whitespace).strip() #Remove excess whitespace
            self.stdout_block += self.line

            #Calculate percentage by parsing the stdout line
            if self.progress_callback:
                self._parse_progress()
                self._parse_transfer_rate()
                self.progress_callback(self.line, self.percent_complete, self.transfer_rate)

        self.stderr_block = '\n'.join(self.p.stderr)

        self.p.poll()

        if self.p.returncode > -1:
            #Set feedback to 100% complete, for cases when no progress is piped from Rsync stdout
            self.progress_callback(None, 100.0, self.transfer_rate)
            return True
        else:
            return False

    def _parse_progress(self):
        #Parse line of stdout. If regex matches, calculate the Sync Progress percentage.
        re_matches = re.findall(r'(to-chk|to-check)=(\d+)/(\d+)', self.line) #e.g. ('to-chk', remaining_files, total_files)
        if re_matches:
            progress_tuple = re_matches[0]
            self.remaining_files = float(progress_tuple[1])
            self.total_files = float(progress_tuple[2])
            self.percent_complete = round(100.0 * (1 - (self.remaining_files/self.total_files)), 2)

    def _parse_transfer_rate(self):
        smoothing_effect = 0.1 #0 < smoothing_effect <= 1, 1 implies no smoothing on output

        rate_tuples = re.findall(r'(\s[0-9]*\.[0-9]+|[0-9]+)(kb|mb|gb|tb)\/s\s', self.line.lower()) #e.g. (1000, 'MB') implies 1000MB/s in tuple format
        rate_conversions = {'kb':3, 'mb':6, 'gb':9, 'tb':12} #rate_tuple * pow(10, value) = bytes/sec
        
        if rate_tuples:
            if rate_tuples[-1][1] in rate_conversions:
                rate_tuple = rate_tuples[-1]
                transfer_rate_sample = float(rate_tuple[0]) * pow(10, rate_conversions[rate_tuple[1]])
                
                #Smoothing Function
                self.transfer_rate = smoothing_effect * transfer_rate_sample + (1.0-smoothing_effect) * self.transfer_rate

    def get_progress(self):
        return self.percent_complete
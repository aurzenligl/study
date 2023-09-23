#!/usr/bin/env python

import re
import requests

# how to get initial list: https://responsive-muse.com/export-youtube-playlist-video-urls-titles-js/
# TODO: process requests in parallel

for x in open('my_data.csv').read().splitlines():
  a, b = x.split(';')
  r = requests.get(b)
  vc = re.search(r'"viewCount":"([0-9]+)"', r.content.decode('utf8'))[1]
  print(';'.join([a, b, vc]))

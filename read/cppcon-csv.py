#!/usr/bin/env python

import re
import requests

# how to get initial list: https://responsive-muse.com/export-youtube-playlist-video-urls-titles-js/
# TODO: process requests in parallel

# Firefox::Ctrl-Shift-C::Console
# COPY & PASTE CODE 1:
# let goToBottom = setInterval(() => window.scrollBy(0, 400), 1000);

# COPY & PASTE CODE 2:
# clearInterval(goToBottom);
# let arrayVideos = [];
# console.log('\n'.repeat(50));
# const links = document.querySelectorAll('a');
# for (const link of links) {
# if (link.id === "video-title") {
#     link.href = link.href.split('&list=')[0];
#     arrayVideos.push(link.title + ';' + link.href);
#     console.log(link.title + '\t' + link.href);
# }
# }

# COPY & PASTE CODE 3:
# let data = arrayVideos.join('\n');
# let blob = new Blob([data], {type: 'text/csv'});
# let elem = window.document.createElement('a');
# elem.href = window.URL.createObjectURL(blob);
# elem.download = 'my_data.csv';
# document.body.appendChild(elem);
# elem.click();
# document.body.removeChild(elem);

for x in open('my_data.csv').read().splitlines():
  a, b = x.split(';')
  r = requests.get(b)
  vc = re.search(r'"viewCount":"([0-9]+)"', r.content.decode('utf8'))[1]
  print(';'.join([a, b, vc]))

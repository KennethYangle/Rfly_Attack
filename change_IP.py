#!/usr/bin/env python3
#coding=utf-8

import glob, re, sys

nargs = len(sys.argv)
# print(sys.argv)
if nargs != 2:
    print("Usage: python3 change_IP.py IP\nFor example: python3 change_IP.py 192.168.110.145")
    sys.exit()

suffix = ["py", "sh", "launch", "bat"]
file_path = [glob.glob('**/*.{}'.format(a), recursive=True) for a in suffix]
file_path = [j for i in file_path for j in i]
# print(file_path)

rtext = sys.argv[1] #获取当前环境参数 即为$search_text 
for f in file_path:
    f1 = open(f,"r",encoding='UTF-8',errors='ignore')
    content = f1.read()
    f1.close()

    t = re.sub(r'192[\.]\d+[\.]\d+[\.]\d+', rtext, content)
    with open(f,"w") as f2:
        f2.write(t)

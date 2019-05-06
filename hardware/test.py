from textwrap import wrap
message_bytes = "03060000000149e8".decode('hex')
strt=message_bytes.encode('hex')

print wrap(strt,2),type(wrap(strt,2))
k=[int(int(i,16)) for i in wrap(strt,2)]
print k
array7=[0x03,0x20,0x10,0x01,0x05]
for i in array7:
    print type(i)
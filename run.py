import subprocess

for f in [1,2,3,4,5]:
    for e in [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1]:
        print "Synthetic Dataset"+str(f)
        tmp = subprocess.Popen(["./Lisco", "-t", "1", "-f", str(f), "-r", "20", "-e", str(e)])
        tmp.wait()
        print tmp

for e in [0.3,0.4,0.7]:
    print "Real Dataset"
        tmp = subprocess.Popen(["./Lisco", "-t", "2", "-f", "1", "-r", "2280", "-e", str(e) ])
        tmp.wait()
        print tmp

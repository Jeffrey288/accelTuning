buff = """
start:[
pos:6.000,0.492,-0.000;
vel:-0.000,-0.000,-0.000;
time:455.496;
]
start:[
pos:6.000,0.492,-0.000;
vel:-0.000,-0.000,-0.000;
time:455.548;
]
"""

buff = ''.join(buff.split())
# print(buff)
while "]" in buff:
    term = buff.find("]")
    # print(term)
    if term == -1:
        pass # wtf this shouldn't happen
    temp = buff[:term+1]
    buff = buff[term+1:]
    # print(temp)
    # print(buff)

    try:
        if temp[:len("start:[")] != "start:[":
            # print(temp[:len("start")])'
            print("continued at start")
            continue
        posInd = temp.find("pos:")
        if posInd == -1:
            print("continued at pos")
            continue
        posTemp = temp[posInd+len("pos:"):temp.find(";",posInd)]
        posTemp = posTemp.split(",")
        posTemp = list(map(float, posTemp))
        
        velInd = temp.find("vel:")
        if velInd == -1:
            print("continued at vel")
            continue
        velTemp = temp[velInd+len("vel:"):temp.find(";",velInd)]
        velTemp = velTemp.split(",")
        velTemp = list(map(float, velTemp))

        timeInd = temp.find("time:")
        if timeInd == -1:
            print("continued at time")
            continue
        timeTemp = temp[timeInd+len("time:"):temp.find(";",timeInd)]
        timeTemp = float(timeTemp)

        print(posTemp, velTemp, timeTemp)
    except:
        print("continued at exception")
        continue

    


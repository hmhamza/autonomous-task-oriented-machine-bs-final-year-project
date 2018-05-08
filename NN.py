# To change this license header, choose License Headers in Project Properties.
# To change this template file, choose Tools | Templates
# and open the template in the editor.

import numpy as np

def bitMapRowGenerator(Map):
    ObstacleLimit=250
    currentNumber=0;
    for j in range(0,17):
        currentNumber=(currentNumber<<1)|(Map[j]<ObstacleLimit)
    return currentNumber

def bitMapGenerator():
    Map=np.loadtxt('ATOM_Map.txt')
    MapDimensions=Map.shape
    BitMap=np.zeros(MapDimensions[0])
    for i in range(0,MapDimensions[0]):
        BitMap[i]=bitMapRowGenerator(Map[i])
    np.savetxt("bitMap.txt",BitMap,"%.0f")
    
def initialNeuralNetworkMapGenerator():
    Map=np.loadtxt('ATOM_Map.txt')
    MapDimensions=Map.shape
    NNMap=np.zeros((MapDimensions[0],MapDimensions[1]*2))
    for i in range(0,MapDimensions[0]):
        for j in range(0,MapDimensions[1]):
            NNMap[i][j*2]=-1
    np.savetxt("neuralNetworkMap.txt",NNMap,'%.0f')

def main():
    #Loading All Map files
    BitMap=np.loadtxt("bitMap.txt")
    NNMap=np.loadtxt("neuralNetworkMap.txt")
    ActualMap=np.loadtxt("ATOM_Map.txt")
    
    #Peforming Localization Multi-Step Process
    MapDimensions=BitMap.shape
    indices=[i for i in range(MapDimensions[0]+1)]
    indices[MapDimensions[0]]=-1
    totalReps=3
    marginForBitError=totalReps-1
    index=0 #Only for readings array
    bitReading=np.zeros(totalReps)
    readings=np.zeros((totalReps,17))
    readings[0]=[255.60975075,255.59353828,256.91890717,259.19675827,7.46178627,313.41123581,311.11717224,311.27119064,3540.16566277,7.14159012,296.71645164,297.05691338,298.49982262,3542.66643524,7.48205185,3527.05788612,7.12537766]
    readings[1]=[254.18305397,3526.08919144,7.58337975,3525.46095848,7.137537,316.49160385,315.43779373,316.13087654,315.74177742,303.24602127,300.08459091,299.28612709,299.492836,300.56285858,303.95936966,7.53068924,3551.38468742]
    readings[2]=[231.55856133,251.90925598,251.76739693,257.22694397,3617.24376678,307.00325966,306.34260178,306.28991127,307.31129646,306.93840981,299.62658882,297.87564278,296.58269882,296.61512375,297.9080677,3604.46023941,6.69980049]
    
    while (marginForBitError>=0):
        ComparedBitValues=np.zeros(MapDimensions[0])
        bitReading[index]=bitMapRowGenerator(readings[index])
        
        counter=0
        i=indices[counter]
        while (i!=-1):
            if (i<MapDimensions[0]):
                ComparedBitValues[i]=17-(bin(int(bitReading[index])^int(BitMap[i])).count('1'))
            counter=counter+1
            i=indices[counter]
        print ComparedBitValues
        
        counter=0
        localmax=np.max(ComparedBitValues)
        for i in range(0,MapDimensions[0]):
            if (ComparedBitValues[i]+marginForBitError>=localmax):
                indices[counter]=i+1
                counter=counter+1
        indices[counter]=-1
        
        marginForBitError=marginForBitError-1
        index=index+1
        
    indices[0]-=1
    print indices[0]
    
    #Learning part of the Map through NN
    resultIndex=indices[0]-2
    noObstacleValue=1010
    obstacleValue=10
    for i in range(0,totalReps):
        bitMapValue=int(BitMap[resultIndex+i])
        value=int(bitReading[i])^bitMapValue
        for j in range(16,-1,-1):
            remainder=value%2
            if (remainder==0):
                NNMap[resultIndex+i][j*2]-=1
                NNMap[resultIndex+i][j*2+1]=0
            else:
                addVal=pow(2,NNMap[resultIndex+i][j*2+1])
                if (NNMap[resultIndex+i][j*2]+addVal>0):
                    NNMap[resultIndex+i][j*2]=-1
                    NNMap[resultIndex+i][j*2+1]=0
                    if (bitMapValue%2==0):
                        ActualMap[resultIndex+i][j]=obstacleValue
                    else:
                        ActualMap[resultIndex+i][j]=noObstacleValue
                    BitMap[resultIndex+i]=bitMapRowGenerator(ActualMap[resultIndex+i])
                else:
                    NNMap[resultIndex+i][j*2]+=addVal
                    NNMap[resultIndex+i][j*2+1]+=1
            value=value>>1
            bitMapValue=bitMapValue>>1
            
    #Saving files back
    np.savetxt("mapT.txt",ActualMap,'%5.5f')
    np.savetxt("bitMapT.txt",BitMap,'%.0f')
    np.savetxt("neuralNetworkMapT.txt",NNMap,'%.0f')
            
main()
print "Done"
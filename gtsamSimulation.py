import numpy as np
import subprocess as sp
import time
import matplotlib.pyplot as plt
import sys

def plotSigmas(sigmaFile,experimentName):

    sigmas = open(sigmaFile, 'r').readlines()

    dx = [[],[],[]]
    dl = [[],[],[]]

    for i in range(len(sigmas)):

        splitLine = sigmas[i].split(' ')

        if i%3 == 0:
            dx[0].append(float(splitLine[0]))
            dl[0].append(float(splitLine[1]))

        elif i%3 == 1:
            dx[1].append(float(splitLine[0]))
            dl[1].append(float(splitLine[1]))

        elif i%3 == 2:
            dx[2].append(float(splitLine[0]))
            dl[2].append(float(splitLine[1]))

    plt.plot(range(0,len(sigmas)/3),dx[0], label='Levenberg Marquart')
    plt.plot(range(0,len(sigmas)/3),dx[1], label='Gauss Newton')
    plt.plot(range(0,len(sigmas)/3),dx[2], label='Non-Linear')
    plt.xlabel('Trial')
    plt.ylabel('State RMSE (m)')
    plt.title(experimentName + ' dl')
    plt.legend()
    plt.savefig('graphs/' + experimentName + 'dl.png')
    plt.close()

    plt.plot(range(0,len(sigmas)/3),dl[0], label='Levenberg Marquart')
    plt.plot(range(0,len(sigmas)/3),dl[1], label='Gauss Newton')
    plt.plot(range(0,len(sigmas)/3),dl[2], label='Non-Linear')
    plt.xlabel('Trial')
    plt.ylabel('Landmark RMSE (m)')
    plt.title(experimentName + ' dx')
    plt.legend()
    plt.savefig('graphs/' + experimentName + 'dx.png')
    plt.close()

def plotResidualsL(residuals, experimentName):

    x = range(0,len(residuals[0])/3)

    plt.plot(x,residuals[0][0:3], label='Levenberg Marquart')
    plt.plot(x,residuals[0][3:6], label='Gauss Newton')
    plt.plot(x,residuals[0][6:9], label='Non-Linear')
    plt.xlabel('Epoch')
    plt.ylabel('Landmark x-Component Error (m)')
    plt.title(experimentName)
    plt.legend()
    plt.savefig('graphs/' + experimentName + 'Lx.png')
    plt.close()

    plt.plot(x,residuals[1][0:3], label='Levenberg Marquart')
    plt.plot(x,residuals[1][3:6], label='Gauss Newton')
    plt.plot(x,residuals[1][6:9], label='Non-Linear')
    plt.xlabel('Epoch')
    plt.ylabel('Landmark y-Component Error (m)')
    plt.title(experimentName)
    plt.legend()
    plt.savefig('graphs/' + experimentName + 'Ly.png')
    plt.close()

def plotResidualsX(residuals, experimentName):

    x = range(0,len(residuals[0])/3)

    plt.plot(x,residuals[0][0:6], label='Levenberg Marquart')
    plt.plot(x,residuals[0][6:12], label='Gauss Newton')
    plt.plot(x,residuals[0][12:18], label='Non-Linear')
    plt.xlabel('Epoch')
    plt.ylabel('State x-Component Error (m)')
    plt.title(experimentName)
    plt.legend()
    plt.savefig('graphs/' + experimentName + 'Xx.png')
    plt.close()

    plt.plot(x,residuals[1][0:6], label='Levenberg Marquart')
    plt.plot(x,residuals[1][6:12], label='Gauss Newton')
    plt.plot(x,residuals[1][12:18], label='Non-Linear')
    plt.xlabel('Epoch')
    plt.ylabel('State y-Component Error (m)')
    plt.title(experimentName)
    plt.legend()
    plt.savefig('graphs/' + experimentName + 'Xy.png')
    plt.close()

    plt.plot(x,residuals[2][0:6], label='Levenberg Marquart')
    plt.plot(x,residuals[2][6:12], label='Gauss Newton')
    plt.plot(x,residuals[2][12:18], label='Non-Linear')
    plt.xlabel('Epoch')
    plt.ylabel('State Theta-Component Error (m)')
    plt.title(experimentName)
    plt.legend()
    plt.savefig('graphs/' + experimentName + 'Xtheta.png')
    plt.close()

def getTimeString(gmtTime):

    timeString = ' '

    for line in gmtTime:

        timeString = timeString + str(line) + ' '

    return timeString

#Takes a synthetic dataset and noise parameters (mean and variance), and applies random gaussian noise to the dataset of
#the given mean and covariance matrix
def generateNoisyData(dataset, noise):

    noiseData = noise.strip().split(' ')
    noisyDataset = []
    cleanDataset = []

    for rawline in dataset:

        line = rawline.strip().split(' ')
        noisyLine = []
        if len(line) > 3:
            #Add noise to the initial poses
            noisyLine.append(line[0])
            noisyLine.append(str(float(line[1]) + np.random.normal(0,noiseData[0])))
            noisyLine.append(str(float(line[2]) + np.random.normal(0,noiseData[1])))
            noisyLine.append(str(float(line[3]) + np.random.normal(0,noiseData[2])))

            #Add noise to the "GPS" observations
            noisyLine.append(str(float(line[4]) + np.random.normal(0,noiseData[3])))
            noisyLine.append(str(float(line[5]) + np.random.normal(0,noiseData[4])))

            #Add noise to the odometry observations
            noisyLine.append(str(float(line[6]) + np.random.normal(0,noiseData[5])))
            noisyLine.append(str(float(line[7]) + np.random.normal(0,noiseData[6])))
            noisyLine.append(str(float(line[8]) + np.random.normal(0,noiseData[7])))

            noisyDataset.append(noisyLine)
            cleanDataset.append(line[1:])

        else:
            noisyLine.append(line[0])
            noisyLine.append(str(float(line[1]) + np.random.normal(0,noiseData[8])))
            noisyLine.append(str(float(line[2]) + np.random.normal(0,noiseData[9])))

            noisyDataset.append(noisyLine)
            cleanDataset.append(line)

    return noisyDataset, cleanDataset

if __name__ == "__main__":
    #Open the synthetic test data, the noise parameters and the output report file
    inputDataFile = open('datasets/' + sys.argv[1] + '.txt','r')
    noiseDataFile = open('datasets/' + sys.argv[2] + '.txt','r')
    reportFile = open('/home/doopy/Documents/gtsamSimulationData/reports/report' + sys.argv[1] + '_' + sys.argv[2] + '.txt','w')
    sigmaFile = open('/home/doopy/Documents/gtsamSimulationData/reports/report' + sys.argv[1] + '_' + sys.argv[2] + 'sigmas.txt','w')

    #Create lists to store the parsed noise data into the experiment labels, the noise that will be applied to the synthetic
    #dataset, and the uncertainties that will be used in the stochastic models
    labels = []
    noise = []
    uncertainties = []

    noisyTestData = None

    for line in noiseDataFile:

        if line[0] == 'l':

            labels.append(line[1:])

        elif line[0] == 'n':

            noise.append(line[1:])

        elif line[0] == 'u':

            uncertainties.append(line[1:])

    #For each experiment
    for i in range(len(noise)):

        experimentTime = getTimeString(time.gmtime())
        #Write the experiment label, the noise and uncertainty parameters to the output file
        reportFile.write('\n' + labels[i].lstrip(' '))
        reportFile.write('Experiment Time:' + experimentTime + '\n')
        reportFile.write('Noise Parameters: ' + noise[i])
        reportFile.write('Uncertainty Parameters: ' + uncertainties[i])
        reportFile.write('Data:\n')
        inputDataFile = open('datasets/' + sys.argv[1] + '.txt','r')

        #Generate the gaussian random noise data from the synthetic dataset and the noise parameters
        noisyTestData, cleanDataset = generateNoisyData(inputDataFile,noise[i])
        noisyTestDataDict = {}

        #Create a file to store the noisy output data
        noisyDataFile = open('/home/doopy/Documents/gtsamSimulationData/datasets/noisyTestData.txt', 'w')
        #Save the noisy data to the output file
        for j in noisyTestData:

            if len(j) > 3:

                noisyDataFile.write("" + j[0] + " " + j[1] + " " + j[2] + " " + j[3] + " " + j[4] + " " + j[5] + " " + j[6] + " " + j[7] + " " + j[8] + "\n")
                reportFile.write("" + j[0] + " " + j[1] + " " + j[2] + " " + j[3] + " " + j[4] + " " + j[5] + " " + j[6] + " " + j[7] + " " + j[8] + "\n")

                noisyTestDataDict[j[0]] = [float(j[1]), float(j[2]), float(j[3])]

            else:

                noisyDataFile.write("" + j[0] + " " + j[1] + " " + j[2] + "\n")
                reportFile.write("" + j[0] + " " + j[1] + " " + j[2] + "\n")

                noisyTestDataDict[j[0]] = [float(j[1]), float(j[2])]

        noisyDataFile.flush()
        noisyDataFile.close()

        #Create a file to store the uncertainties for the experiment
        uncertaintyFile = open('/home/doopy/Documents/gtsamSimulationData/datasets/uncertainties.txt', 'w')

        #Write the stochastic model to the file
        uncertaintyFile.write(uncertainties[i])

        uncertaintyFile.flush()
        uncertaintyFile.close()

        #Run the gtsamSim experiment
        sp.call("./gtsamSim")

        resultsFile = open('/home/doopy/Documents/gtsamSimulationData/results/results.txt')

        reportFile.write("Results:\n")

        index = 0
        residualsL = [[],[]]
        residualsX = [[],[],[]]

        sumSquareErrorL = [0.0,0.0]
        sumSquareErrorX = [0.0,0.0,0.0]

        title = None

        labelIter = iter(labels)

        for line in resultsFile:

            splitLine = line.split()

            if splitLine[1][0] == '(':

                if splitLine[0][0] == 'l':

                    x = float(splitLine[1].strip('(').strip(','))
                    y = float(splitLine[2].strip(')').strip(','))

                    reportFile.write(splitLine[0] + ' ' + str(x) + ' ' + str(y) + '\n')
                    reportFile.write(str(x - noisyTestDataDict[splitLine[0]][0]) + ' ' + str(y - noisyTestDataDict[splitLine[0]][1]) + ' ' + '\n')
                    residualsL[0].append(x - noisyTestDataDict[splitLine[0]][0])
                    residualsL[1].append(y - noisyTestDataDict[splitLine[0]][1])
                    sumSquareErrorL[0] += (x - noisyTestDataDict[splitLine[0]][0])**2
                    sumSquareErrorL[1] += (y - noisyTestDataDict[splitLine[0]][1])**2

                elif splitLine[0][0] == 'x' and splitLine[0][1] == '5':

                    x = float(splitLine[1].strip('(').strip(','))
                    y = float(splitLine[2].strip(')').strip(','))
                    theta = float(splitLine[3])
                    sumSquareErrorX[0] += (x)**2
                    sumSquareErrorX[1] += (y)**2
                    sumSquareErrorX[2] += (theta)**2
                    residualsX[0].append(x)
                    residualsX[1].append(y)
                    residualsX[2].append(theta)
                    reportFile.write(splitLine[0] + ' ' + str(x) + ' ' + str(y) + str(theta) + '\n')
                    reportFile.write('dLx = ' + str(sumSquareErrorL[0]**(0.5)) + ' dLy = ' + str(sumSquareErrorL[1]**(0.5)) + '\n')
                    reportFile.write('dL = ' + str((sumSquareErrorL[0] + sumSquareErrorL[1])**(0.5)) + '\n')
                    sigmaFile.write(str((sumSquareErrorL[0] + sumSquareErrorL[1])**(0.5)) + ' ')
                    reportFile.write('dXx = ' + str(sumSquareErrorX[0]**(0.5)) + ' dXy = ' + str(sumSquareErrorX[1]**(0.5)) + ' dXz = ' + str(sumSquareErrorX[2]**(0.5)) + '\n')
                    reportFile.write('dX = ' + str((sumSquareErrorX[0] + sumSquareErrorX[1] + sumSquareErrorX[2])**(0.5)) + '\n')
                    sigmaFile.write(str((sumSquareErrorX[0] + sumSquareErrorX[1] + sumSquareErrorX[2])**(0.5)) + '\n')

                elif splitLine[0][0] == 'x' and splitLine[0][1] != '0':

                        x = float(splitLine[1].strip('(').strip(','))
                        y = float(splitLine[2].strip(')').strip(','))
                        theta = float(splitLine[3])
                        residualsX[0].append(x - noisyTestDataDict[splitLine[0][1]][0])
                        residualsX[1].append(y - noisyTestDataDict[splitLine[0][1]][1])
                        residualsX[2].append(theta - noisyTestDataDict[splitLine[0][1]][2])

                        sumSquareErrorX[0] += (x - noisyTestDataDict[splitLine[0][1]][0])**2
                        sumSquareErrorX[1] += (y - noisyTestDataDict[splitLine[0][1]][1])**2
                        sumSquareErrorX[2] += (theta - noisyTestDataDict[splitLine[0][1]][2])**2

                        reportFile.write(splitLine[0] + ' ' + str(x) + ' ' + str(y) + str(theta) + '\n')
                        reportFile.write(str(x - noisyTestDataDict[splitLine[0][1]][0]) + ' ' + str(y - noisyTestDataDict[splitLine[0][1]][1]) + ' ' + str(theta - noisyTestDataDict[splitLine[0][1]][2]) + '\n')


                elif splitLine[0][0] == 'x' and splitLine[0][1] == '0':

                    x = float(splitLine[1].strip('(').strip(','))
                    y = float(splitLine[2].strip(')').strip(','))
                    theta = float(splitLine[3])
                    sumSquareErrorX[0] += (x)**2
                    sumSquareErrorX[1] += (y)**2
                    sumSquareErrorX[2] += (theta)**2
                    residualsX[0].append(x)
                    residualsX[1].append(y)
                    residualsX[2].append(theta)
                    reportFile.write(splitLine[0] + ' ' + str(x) + ' ' + str(y) + str(theta) + '\n')

            elif splitLine[0] == 'end':

                #plotResidualsL(residualsL, sys.argv[1] + '_' + sys.argv[2] + '_' + labels[i])
                #plotResidualsX(residualsX, sys.argv[1] + '_' + sys.argv[2] + '_' + labels[i])
                residualsL = [[],[]]
                residualsX = [[],[],[]]

                sumSquareErrorL = [0.0,0.0]
                sumSquareErrorX = [0.0,0.0,0.0]

            else:

                reportFile.write('\n' + line)
                title = line


    sigmaFile.close()
    plotSigmas('/home/doopy/Documents/gtsamSimulationData/reports/report' + sys.argv[1] + '_' + sys.argv[2] + 'sigmas.txt',sys.argv[1] + '_' + sys.argv[2])
    print 'Done'

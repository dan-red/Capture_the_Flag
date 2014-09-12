from pyrobot.engine import Engine
from pyrobot.system.config import *
import time
import os
import sys
import math, random
import cPickle as pickle
from neat import config, population, chromosome, genome, visualize
from neat.nn import nn_pure as nn

def eval_fitness(population):
    """
    Evaluate the fitness of the given population.
    """
    for chromo in population:
        chromo.fitness = eval_individual(chromo)
        print "Fitness:", chromo.fitness, "\n"

def eval_individual(chromo, currentGen, logFP=None):
    if currentGen > 0:
        chromo_file = "blue_best_chromo_" + str(currentGen-1)
        while not os.path.isfile(chromo_file):
            pass

        time.sleep(1)
    else:
        chromo_file = "blue_init_chromo"
    fp = open(chromo_file, "r")
    opponentChromo = pickle.load(fp)
    fp.close()
    redBrain = nn.create_ffphenotype(chromo)
    blueBrain = nn.create_ffphenotype(opponentChromo)
    trials = 2
    steps = 125
    score = 0
    for trial in range(trials):
        redEng.simulation[0].setPose('redRobot', 5, 1, 0)
        redEng.simulation[0].setPose('blueRobot', 5, 9, math.pi)
        redEng.simulation[0].eval("self.refillLights(0.3)")
        
        redLight = 0
        blueLight = 0
        redCounter = 0
        blueCounter = 0
        accomplished = 0
        
        for i in range(steps):
            redBrain.flush()
            blueBrain.flush()
            redEng.update()
            blueEng.update()
            redResult = redEng.eat(-1)
            blueResult = blueEng.eat(-1)
            if redResult > 0:
                redLight = 1
                redCounter = 1

                if logFP:
                    logFP.write("Red robot has the ball\n")

            elif blueResult > 0:              
                blueLight = 1
                blueCounter = 1

                if logFP:
                    logFP.write("Blue robot has the ball\n")
             
            redPosition = redEng.simulation[0].getPose('redRobot')
            bluePosition = redEng.simulation[0].getPose('blueRobot')

            xdist = redPosition[0] - bluePosition[0]
            ydist = redPosition[1] - bluePosition[1]
            dist = math.sqrt(xdist**2 + ydist**2)

            if dist < 0.75:
              if redLight > 0:
                redLight = 0
                blueLight = 1
                redEng.simulation[0].setPose('redRobot', 5, 1, 0)

              elif blueLight > 0:
                redLight = 1
                blueLight = 0
                redEng.simulation[0].setPose('blueRobot', 5, 9, math.pi)

            if redEng.stall:
                break
            
            sonar = min([s.distance() for s in redEng.range["front"]])
            xdistFromBase = redPosition[0] - 5
            ydistFromBase = redPosition[1] - 1
            heading = redPosition[2]

            xdistFromOpponent = xdist
            ydistFromOpponent = ydist
            opponentHeading = bluePosition[2]


            redIn = redEng.light[0].value
            redIn.append(sonar/10.0)
            redIn.append(redLight)
            redIn.append(blueLight)
            redIn.append(xdistFromBase)
            redIn.append(ydistFromBase)
            redIn.append(heading)
            redIn.append(xdistFromOpponent)
            redIn.append(ydistFromOpponent)
            redIn.append(opponentHeading)
            

            # print "redIn:", redIn
            redOut = redBrain.sactivate(redIn)

            
            bluesonar = min([s.distance() for s in blueEng.range["front"]])
            bluexdistFromBase = bluePosition[0] - 5
            blueydistFromBase = bluePosition[1] - 9
            blueheading = bluePosition[2]

            bluexdist = bluePosition[0] - redPosition[0]
            blueydist = bluePosition[1] - redPosition[1]

            bluexdistFromOpponent = bluexdist
            blueydistFromOpponent = blueydist
            blueopponentHeading = bluePosition[2]

            blueIn = blueEng.light[0].value
            blueIn.append(bluesonar/10.0)
            blueIn.append(redLight)
            blueIn.append(blueLight)
            blueIn.append(bluexdistFromBase)
            blueIn.append(blueydistFromBase)
            blueIn.append(blueheading)
            blueIn.append(bluexdistFromOpponent)
            blueIn.append(blueydistFromOpponent)
            blueIn.append(blueopponentHeading)


            # print "blueIn:", blueIn
            blueOut = blueBrain.sactivate(blueIn)

            distFromBase = math.sqrt(xdistFromBase**2 + ydistFromBase**2)
            if redLight and distFromBase < 0.5:
              score += 24
              accomplished = 1
              break
            
            # took out ".robot" between redEng and motors, e.g.
            if redLight:
              redEng.motors(0.7*redOut[0], 0.7*redOut[1])
              blueEng.motors(blueOut[0], blueOut[1])

            elif blueLight:
              blueEng.motors(0.7*blueOut[0], 0.7*blueOut[1])
              redEng.motors(redOut[0], redOut[1])

            else:
              redEng.motors(redOut[0], redOut[1])
              blueEng.motors(blueOut[0], blueOut[1])
    
        if redCounter == 0 and blueCounter == 0 and not accomplished:
          xdistFromLight = redPosition[0] - 5
          ydistFromLight = redPosition[1] - 5
          distFromLight = math.sqrt(xdistFromLight**2 + ydistFromLight**2)
          score += 8 - distFromLight         # fitness between 1 and 8 (approx)

        elif redCounter == 0 and not accomplished:
          distFromOpponent = math.sqrt(xdistFromOpponent**2 + ydistFromOpponent**2)
          score += 16 - distFromOpponent

        elif redCounter == 1 and redLight == 1 and not accomplished:
          distFromBase = math.sqrt(xdistFromBase**2 + ydistFromBase**2)
          score += 24 - distFromBase

        elif not accomplished:
          distFromOpponent = math.sqrt(xdistFromOpponent**2 + ydistFromOpponent**2)
          score += 20 - distFromOpponent

    blueEng.stop()
    redEng.stop()
    return score

# start up the simulation
pyroConfig = Configuration()
pyroConfig.put("pyrobot", "gui", "tk")
redRobot = Engine(robotfile="PyrobotRobot60000.py",
                 simfile="PyrobotSimulator",
                 config=pyroConfig,
                 worldfile="redFlagWorld.py")
blueRobot = Engine(robotfile="PyrobotRobot60001.py")
redEng = redRobot.robot
blueEng = blueRobot.robot

# set up neat
config.load('redEat_config')
chromosome.node_gene_type = genome.NodeGene

def main(argv = None):
    if argv is None:
        argv = sys.argv[1:]
    chromo_file = None
    log_file = None
    if len(argv) == 0:
        pass
    elif len(argv) == 1:
        chromo_gen = argv[0]
        chromo_file = "red_best_chromo_" + chromo_gen
    elif len(argv) == 2:
        chromo_gen = argv[0]
        chromo_file = "red_best_chromo_" + chromo_gen
        log_file = argv[1]
    else:
        print "Usage: python eat.py [chromo_gen] [log_file]"
        return

    if chromo_file:
        if log_file:
            logFP = open(log_file, "w")
        else:
            logFP = None
        # Test an already evolved network
        fp = open(chromo_file, "r")
        chromo = pickle.load(fp)
        fp.close()
        print chromo
        result = eval_individual(chromo, int(chromo_gen), logFP)
        print "Fitness:", result
        if log_file:
            logFP.write("Fitness %f" % result)
            logFP.close()

if __name__ == '__main__':
    main()


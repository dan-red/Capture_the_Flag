from pyrobot.engine import Engine
from pyrobot.system.config import *
import time
import os
import sys
import math, random
import cPickle as pickle
from neat import config, population, chromosome, genome, visualize
from neat.nn import nn_pure as nn

config.load('blueEat_config')
chromosome.node_gene_type = genome.NodeGene
pyroConfig = Configuration()
pyroConfig.put("pyrobot", "gui", "tk")
redEng = Engine(robotfile="PyrobotRobot60002.py",
                simfile="PyrobotSimulator",
                config=pyroConfig,
                worldfile="blueFlagWorld.py")
blueEng = Engine(robotfile="PyrobotRobot60003.py")

def eval_fitness(population):
    """
    Evaluate the fitness of the given population.
    """
    print "Evaluating the BLUE robot..."
    for chromo in population:
        chromo.fitness = eval_individual(chromo, population.getGen())
        print "Fitness:", chromo.fitness

def eval_individual(chromo, currentGen, logFP=None):
    if currentGen > 0:
        chromo_file = "red_best_chromo_" + str(currentGen-1)
        while not os.path.isfile(chromo_file):
            pass
        time.sleep(1)
    else:
        chromo_file = "red_init_chromo"
    fp = open(chromo_file, "r")
    opponentChromo = pickle.load(fp)
    fp.close()
    redBrain = nn.create_ffphenotype(opponentChromo)
    blueBrain = nn.create_ffphenotype(chromo)
    trials = 2
    steps = 125
    score = 0
    for trial in range(trials):
        redEng.robot.simulation[0].setPose('redRobot', 5, 1, 0)
        redEng.robot.simulation[0].setPose('blueRobot', 5, 9, math.pi)
        redEng.robot.simulation[0].eval("self.refillLights(0.3)")

        redLight = 0
        blueLight = 0
        redCounter = 0
        blueCounter = 0
        accomplished = 0

        for i in range(steps):
            redBrain.flush()
            blueBrain.flush()
            redEng.robot.update()
            blueEng.robot.update()
            redResult = redEng.robot.eat(-1)
            blueResult = blueEng.robot.eat(-1)
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

            redPosition = redEng.robot.simulation[0].getPose('redRobot')
            bluePosition = redEng.robot.simulation[0].getPose('blueRobot')

            xdist = redPosition[0] - bluePosition[0]
            ydist = redPosition[1] - bluePosition[1]
            dist = math.sqrt(xdist**2 + ydist**2)

            if dist < 0.75:
              if redLight > 0:
                redLight = 0
                blueLight = 1
                redEng.robot.simulation[0].setPose('redRobot', 5, 1, 0)

              elif blueLight > 0:
                redLight = 1
                blueLight = 0
                redEng.robot.simulation[0].setPose('blueRobot', 5, 9, math.pi)

            if blueEng.robot.stall:
                break
            
            sonar = min([s.distance() for s in redEng.robot.range["front"]])
            xdistFromBase = redPosition[0] - 5
            ydistFromBase = redPosition[1] - 1
            heading = redPosition[2]

            xdistFromOpponent = xdist
            ydistFromOpponent = ydist
            opponentHeading = bluePosition[2]


            redIn = redEng.robot.light[0].value
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

            
            bluesonar = min([s.distance() for s in blueEng.robot.range["front"]])
            bluexdistFromBase = bluePosition[0] - 5
            blueydistFromBase = bluePosition[1] - 9
            blueheading = bluePosition[2]

            bluexdist = bluePosition[0] - redPosition[0]
            blueydist = bluePosition[1] - redPosition[1]

            bluexdistFromOpponent = bluexdist
            blueydistFromOpponent = blueydist
            blueopponentHeading = bluePosition[2]


            blueIn = blueEng.robot.light[0].value
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

            bluedistFromBase = math.sqrt(bluexdistFromBase**2 + blueydistFromBase**2)
            if blueLight and bluedistFromBase < 0.5:
              score += 24
              accomplished = 1
              break
            
            if redLight:
              redEng.robot.motors(0.7*redOut[0], 0.7*redOut[1])
              blueEng.robot.motors(blueOut[0], blueOut[1])

            elif blueLight:
              blueEng.robot.motors(0.7*blueOut[0], 0.7*blueOut[1])
              redEng.robot.motors(redOut[0], redOut[1])

            else:
              redEng.robot.motors(redOut[0], redOut[1])
              blueEng.robot.motors(blueOut[0], blueOut[1])
    
        if redCounter == 0 and blueCounter == 0 and not accomplished:
          bluexdistFromLight = bluePosition[0] - 5
          blueydistFromLight = bluePosition[1] - 5
          bluedistFromLight = math.sqrt(bluexdistFromLight**2 + blueydistFromLight**2)
          score += 8 - bluedistFromLight         # fitness between 1 and 8 (approx)

        elif blueCounter == 0 and not accomplished:
          bluedistFromOpponent = math.sqrt(bluexdistFromOpponent**2 + blueydistFromOpponent**2)
          score += 16 - bluedistFromOpponent

        elif redCounter == 1 and redLight == 1 and not accomplished:
          bluedistFromBase = math.sqrt(bluexdistFromBase**2 + blueydistFromBase**2)
          score += 24 - bluedistFromBase

        elif not accomplished:
          bluedistFromOpponent = math.sqrt(bluexdistFromOpponent**2 + blueydistFromOpponent**2)
          score += 20 - bluedistFromOpponent


    blueEng.robot.stop()
    redEng.robot.stop()
    return score




def main(argv = None):
    if argv is None:
        argv = sys.argv[1:]
    chromo_file = None
    log_file = None
    if len(argv) == 0:
        pass
    elif len(argv) == 1:
        chromo_gen = argv[0]
        chromo_file = "blue_best_chromo_" + chromo_gen
    elif len(argv) == 2:
        chromo_gen = argv[0]
        chromo_file = "blue_best_chromo_" + chromo_gen
        log_file = argv[1]
    else:
        print "Usage: python blueEat.py [chromo_gen] [log_file]"
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
    else:
        # Start the evolutionary process
        population.Population.evaluate = eval_fitness
        pop = population.Population()
        pop.epoch(23, report=True, save_best=True, checkpoint_interval=4, \
                  checkpoint_generation=None, name="blue")
        # changed from 4, None, 2
    
        # Plots the evolution of the best/average fitness (requires Biggles)
        visualize.plot_stats(pop.stats, name="blue")
        # Visualizes speciation
        visualize.plot_species(pop.species_log, name="blue")
        winner = pop.best
        # Visualize the winner network (requires PyDot)
        visualize.draw_net(winner, "blue") # best chromosome
        print "NEAT evolution complete"

if __name__ == '__main__':
    main()


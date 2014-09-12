import cPickle as pickle
from neat import config, population, chromosome, genome

config.load('blueEat_config')
chromosome.node_gene_type = genome.NodeGene
pop = population.Population()
chromo = pop.create_individual()
fp = open("blue_init_chromo", 'w')
pickle.dump(chromo, fp)

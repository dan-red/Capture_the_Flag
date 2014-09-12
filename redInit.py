import cPickle as pickle
from neat import config, population, chromosome, genome

config.load('redEat_config')
chromosome.node_gene_type = genome.NodeGene
pop = population.Population()
chromo = pop.create_individual()
fp = open("red_init_chromo", 'w')
pickle.dump(chromo, fp)

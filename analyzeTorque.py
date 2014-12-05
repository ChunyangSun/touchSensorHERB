# visualization
import matplotlib.pyplot as plt
import numpy as np 
import sys
from scipy.spatial.distance import pdist

def visualize1(tor_list):

    plt.figure(2)
    plt.subplot(711)
    plt.plot(tor_list[0], 'bo')
    
    plt.subplot(712)
    plt.plot(tor_list[1], 'bo')
    
    plt.subplot(713)
    plt.plot(tor_list[2], 'bo')
    
    plt.subplot(714)
    plt.plot(tor_list[3], 'bo')
    
    plt.subplot(715)
    plt.plot(tor_list[4], 'bo')
    
    plt.subplot(716)
    plt.plot(tor_list[5], 'bo')
    
    plt.subplot(717)
    plt.plot(tor_list[6], 'bo')
    plt.show()

def compare(model, data):
    ''' find out the smallest distance with the model '''

    l_m = len(model)
    data = np.array(data)
    model = np.array(model)
    min_var = 500 
    var = []
    print len(data)
    print len(model)
    mn_m = [np.mean(model) in xrange(l_m)]


    for i in xrange(len(data)- len(model)):
        d = data[i: i+l_m]   
        mn_d = [np.mean(d) in xrange(l_m)]
        # two_d_array = zip(d, model)

        # print two_d_array
        # if i ==3520:
        #     print data[i:i+10]
        #     print model[0:10]

        #     print pdist([model-mn_m, d - mn_d], 'euclidean')

        var.append(pdist([model-mn_m, d - mn_d], 'euclidean'))
        
        # print vazip(model-mn_m, d - mn_d)r 
        if min_var > var[i]:
            min_var = var[i]
            min_idx = i 
    print var[3521:3530]
    print max(var)
    # print min_idx
    # IPython.embed()
    # visualize1(data[min_idx : min_idx + l_m])
    visualize1(var)


def readfile():
    tor_list = [[] for i in xrange(7)]
    j = 0
    data = []

    f = open('data/torque', 'r')
    
    for line in f:
        j+= 1
        # if j > 9600 and j < 9900:
        # if j == 5812:b
        
        #     visualize1(tor_list)
        # tor = line[1:-2].split(', ')
        # print tor
        # for item in xrange(len(tor)):
        #     tor_list[0].append(float(item))

# for pub 
        if line[0:6] == "data: ":
            tor = line[7:-2].split(', ')

            for i in xrange(len(tor)):
                tor_list[i].append(float(tor[i]))
    visualize1(tor_list)
                   
            # model = line[2:-3].split(', ')[3550:3600]
            # tor = line[2:-3].split(', ')


            # for i in xrange(len(tor)):
            #     data.append(float(tor[i]))

            # for i in xrange(len(model)):
            #     model[i] = float(model[i])
            # f1 = open('sp', 'w')
            # f1.write(str(model))


            # visualize1(model)
            # visualize1(data)
            # compare(model, data)

# for data files 
        # tor_list[i].append(float(line))

    #     tor = line[1:-2].split(', ')
    #     for i in xrange(len(tor)):
    #         tor_list[i].append(float(tor[i]))
    # visualize1(tor_list)
  

if __name__ == '__main__':
    readfile()
import numpy as np
import matplotlib.pyplot as plt
import os

legend = []
tempDir = os.listdir("temp")

for file in tempDir:
    
    name = file.split("_")
    legend = f"{name[0]} Creatures, {name[1]} Generations, {name[2][0]} Seed"
    
    with open(f"temp/{file}") as f:
        data = []
        for line in f:
            num = line.split("\n")[0]
            if num != '':
                data.append(float(num))
        plt.plot(data, label=legend)
        #plt.plot(data)

#plt.gca().invert_yaxis()
plt.ylabel("Fitness")
plt.xlabel("Generation Number")
plt.title("Fitness Graph:")
plt.legend()

graphNum = len(os.listdir("graphs"))
plt.savefig(f"graphs/Graph{graphNum}.png")
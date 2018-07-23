#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

sns.set_style("whitegrid")
sns.set_color_codes("muted")

x = np.random.randn(1000)

sns.distplot(x)

#plt.hist(gaussian_numbers)
#plt.title("Gaussian Histogram")
#plt.xlabel("Value")
#plt.ylabel("Frequency")

fig = plt.gcf()

sns.despine(left=True, bottom=True)

plt.show()

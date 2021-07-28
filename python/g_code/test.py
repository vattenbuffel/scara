from matplotlib import pyplot
from matplotlib import patches

fig, ax = pyplot.subplots()

arc = patches.Arc(
    xy=(2, 3),
    width=5,
    height=5,
    angle=0,
    theta1=45,
    theta2=180,
)

ax.add_patch(arc)
ax.set_aspect('equal')
ax.autoscale()
pyplot.show()
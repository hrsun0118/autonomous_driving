# Survey for partner preferences
# extra credit
# HW3 extension
import matplotlib.pyplot as plt
import numpy as np

plt.ion()
def axes():
    """ Prepare standard axes with arrows """
    plt.figure()
    plt.gca().axis('equal')
    plt.xlim((-7,7))
    plt.ylim((-7,7))


def plot(f):
    """ Make a 2-D quiver plot of the field f """
    axes()
    plt.quiver([0,-5],[-5,0],[0,10],[10,0],angles='xy', scale_units='xy', scale=1)
    for x1 in range(-4,6,2):
        for x2 in range(-4,6,2):
            x=np.array([[x1],[x2]])
            fx=f(x)
            plt.quiver(x[0],x[1],fx[0],fx[1],color='blue',angles='xy', scale_units='xy', scale=1)

def to_euler(f):
    """ Create a Euler iteration function from a field  """
    return lambda x,h: x+h*f(x)

def euler_run(euler_step,h,x0,nb_steps,u=None,line_color='red'):
    """ Run Euler step iteration functions, and collect first two coordinates """
    x_all=x0[0:2,:]
    xk=x0
    for step in range(0,nb_steps):
        if u:
            xk=euler_step(xk,u,h)
        else:
            xk=euler_step(xk,h)
        x_all=np.hstack((x_all,xk[0:2,:]))
    plt.plot(x_all[0],x_all[1],color=line_color)

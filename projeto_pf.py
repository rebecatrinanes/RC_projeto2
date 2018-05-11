#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Esta classe deve conter todas as suas implementações relevantes para seu filtro de partículas
"""

from pf import Particle, create_particles, draw_random_sample
import numpy as np
import inspercles # necessário para o a função nb_lidar que simula o laser
import math
from scipy.stats import norm


largura = 775 # largura do mapa
altura = 748  # altura do mapa

# Robo
robot = Particle(largura/2, altura/2, math.pi/4, 1.0)


num_particulas = 500


# Os angulos em que o robo simulado vai ter sensores
angles = np.linspace(0.0, 2*math.pi, num=8, endpoint=False)

# Lista mais longa
movimentos_longos = [[-10, -10, 0], [-10, 10, 0], [-10,0,0], [-10, 0, 0],
              [0,0,math.pi/12.0], [0, 0, math.pi/12.0], [0, 0, math.pi/12],[0,0,-math.pi/4],
              [-5, 0, 0],[-5,0,0], [-5,0,0], [-10,0,0],[-10,0,0], [-10,0,0],[-10,0,0],[-10,0,0],[-15,0,0],
              [0,0,-math.pi/4],[0, 10, 0], [0,10,0], [0, 10, 0], [0,10,0], [0,0,math.pi/8], [0,10,0], [0,10,0], 
              [0,10,0], [0,10,0], [0,10,0],[0,10,0],
              [0,0,-math.radians(90)],
              [math.cos(math.pi/3)*10, math.sin(math.pi/3),0],[math.cos(math.pi/3)*10, math.sin(math.pi/3),0],[math.cos(math.pi/3)*10, math.sin(math.pi/3),0],
              [math.cos(math.pi/3)*10, math.sin(math.pi/3),0]]

# Lista curta
movimentos_curtos = [[-10, -10, 0], [-10, 10, 0], [-10,0,0], [-10, 0, 0]]

movimentos_relativos = [[0, -math.pi/3],[10, 0],[10, 0], [10, 0], [10, 0],[15, 0],[15, 0],[15, 0],[0, -math.pi/2],[10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [0, -math.pi/2], 
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [0, -math.pi/2], 
                       [10,0], [0, -math.pi/4], [10,0], [10,0], [10,0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0]]



movimentos = movimentos_relativos



def cria_particulas(minx=0, miny=0, maxx=largura, maxy=altura, n_particulas=num_particulas):

  particulas = []

  for i in range(n_particulas):

    x = np.random.uniform(minx, maxx)
    y = np.random.uniform(miny, maxy)
    theta = np.random.uniform(0, 2*math.pi)
    p = Particle(x, y, theta,  w=1.0) 
    particulas.append(p)
        
  return  particulas
    
def move_particulas(particulas, movimento):
  
  for i in range(len(particulas)):
    
    dp = np.random.normal(0, 1.5) #desvio padrão
    dt = np.random.normal(0, math.radians(2)) #desvio do ângulo
    linear = movimento[0] + dp 
    angular = movimento[1] + dt
    particulas[i].move_relative([linear,angular]) #aplicando a função em todas as partículas com os respectivos desvios      
           
  return particulas
    

def leituras_laser_evidencias(robot, particulas):

  leitura_robo = inspercles.nb_lidar(robot, angles)
  sigma = 9
  w_all = 0.0

  for i in range(len(particulas)):
    prob = 0.0
    prob_total = 0.0
    
    leitura_particula = inspercles.nb_lidar(particulas[i], angles)

    for k in leitura_robo:

      zjchp = leitura_robo[k]
      zj = leitura_particula[k]
      
      prob = norm.pdf(zj, loc=zjchp, scale=sigma)
      
      prob_total += prob #prob dos 8 raios de cada partícula
    
    #print(prob)  
    particulas[i].w *= prob_total #guardar prob dos 8 raios em p.w

    w_all += particulas[i].w #soma todos os valores de w

  alpha = 1/w_all

  for p in particulas:
    p.w *= alpha
    
    
def reamostrar(particulas, n_particulas = num_particulas):

  particulas_pesos = [p.w for p in particulas]

  novas_particulas = draw_random_sample(particulas, particulas_pesos, n_particulas)

  for p in novas_particulas:
    
    dx = np.random.normal(0, 10)
    dy = np.random.normal(0, 10)
    dtheta = np.random.normal(0, math.radians(15))
    p.x += dx
    p.y += dy
    p.theta += dtheta
    
  for p in novas_particulas:
    p.w = 1


  return novas_particulas

    








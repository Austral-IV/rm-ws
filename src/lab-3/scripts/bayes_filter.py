import numpy as np
from likelihood_fields import Localizacion
import time

class BayesFilter():
    loc = Localizacion()
    zmax, pxperm = (4, 100)
    X = zmax*pxperm
    def __init__(self, arr, accuracy = 0.9):
        self.prior = np.ones([len(arr), self.X])/len(arr) #inic. prob uniforme
        self.z = accuracy # ranger_accuracy
        self.ranges = np.array([])
        self.time = 0
    def update_belief(self, zt, prior):
        # quizá self.z debería ser likelihood_model(zt, x)
        # correct_scale = self.z/(1-self.z)
        # prior[self.ranges == zt] *= correct_scale # calculamos likelyhood
        # return prior/sum(prior) # normalizammos

        """ lo mismo pero usando la función de likelihood de daniel: """
        lh = np.array([])
        self.timer_st()
        for i in prior:
            # calcula el likelyhood para cada distancia
            q = self.loc.likelihood_fields_model(zt, [0, 0, 0])
            np.append(lh, q/(1-q))
        self.timer_end("tiempo en calc likelihoods: ")
        
        return (lh*prior) / sum(lh*prior)
    
    def timer_st(self):self.time = time.time()
    def timer_end(self, msg=None): 
        print(msg, time.time() - self.time)
            

    def bayes_filter(self, zt, ut=1):
        """ para cada valor de distancia entregado debemos calcular el belief
        ut (i.e. acción) es la noacción: p(xt|ut, xt-1) = 1 si xt=xt-1 si no 0"""
        X = self.X
        bel = []
        correct_scale = self.z*10
        posterior = np.array([])
        # likelihood = [self.loc.likelihood_fields_model(np.array(z), x) for z,x in zip(, )]
        for i, belief in enumerate(self.prior):
            # para cada distancia, si coincide con la medición, aumenta su certeza
            
            np.append(posterior, self.update_belief(zt[i], belief))
        
        self.ranegs = np.array([])
        self.prior = posterior
        for post in self.prior:
            np.append(self.ranges, np.where(post == max(post))[0])
        
if __name__ == '__main__':

    z = [0.521]     # Distancia real: 0.52
    x = [1.18, 1.36, 0]
    filter = BayesFilter(x)
    filter.bayes_filter([z, z, z])
    
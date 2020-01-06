#!/usr/bin/env python
# -*- coding: utf-8 -*-

import torch
import numpy as np


class GaussianProcessImplicitSurfaces:
    def __init__(self, X, Y, kernel, sigma=0.2, m=1, c=100, z_limit=0.03):

        self.X  = X
        self.Y  = Y

        # self.sigma   = torch.tensor(np.log(sigma))
        self.sigma   = sigma
        self.m       = m
        self.c       = c
        self.z_limit = z_limit
        self.kernel  = kernel

        # print self.task_kernel_params
        self.KX = self.kernel(self.X, self.X)
        self.Y_m = self.Y - self.m

        L  = torch.cholesky(self.KX + torch.eye(self.X.size()[0]) * torch.exp(self.sigma))

        self.invL = torch.solve(torch.eye(L.size()[0]), L)[0]
        self.invK = torch.mm(self.invL.T, self.invL)

    def predict(self, x):
        Kx  = ( torch.ones(len(x)) * (1.0 / float(self.kernel.params[0])) )[:, None]
        KXx = self.kernel(self.X, x)

        # sum ( A * B.T, axis=0 ) = diag( A.dot(B) )
        invL_k = self.invL.mm(KXx)

        mean = self.m + KXx.T.mm(self.invK).mm(self.Y_m)
        cov  = Kx - torch.sum(invL_k * invL_k, dim=0)[:, None]

        return mean, cov

    def compute_grad(self, flag):
        self.kernel.params.requires_grad = flag
        # self.sigma.requires_grad = flag

    def log_likehood(self):

        KX = self.kernel(self.X, self.X)
        K = KX + torch.eye(self.X.size()[0]) * torch.exp(self.sigma)

        invKY = torch.solve(self.Y_m, K)[0]
        # import ipdb; ipdb.set_trace()

        return  torch.logdet(K) + self.Y_m.T.mm(invKY)

    def learning(self):
        max_iter = 500

        self.compute_grad(True)
        # params = [self.task_kernel_params]
        # params = [self.learning_params]
        params = [self.kernel.params] + [self.sigma]
        optimizer = torch.optim.Adam(params, lr=0.001)
        # optimizer = torch.optim.LBFGS(params, history_size=20, max_iter=100)
        # optimizer = torch.optim.SGD(params, lr=1e-3)

        print "params:", params
        for i in range(max_iter):
            optimizer.zero_grad()
            f = self.log_likehood()

            f.backward()

            if i % 10 == 0:
                print "-------------------------------------"
                # print self.learning_params.grad
                print f.item()
            # def closure():
            #     optimizer.zero_grad()
            #     with torch.autograd.detect_anomaly():
            #         f = self.log_likehood()
            #         f.backward()
            #     return f
            # optimizer.step(closure)
            optimizer.step()

        self.compute_grad(False)

        self.KX = self.kernel(self.X, self.X)
        L  = torch.cholesky(self.KX + torch.eye(self.X.size()[0]) * torch.exp(self.sigma))

        self.invL = torch.solve(torch.eye(L.size()[0]), L)[0]
        self.invK = torch.mm(self.invL.T, self.invL)

    def predict_direction(self, x):
        diff_Kx = self.kernel.derivative(x, self.X)
        KXx     = self.kernel(self.X, x)

        diff_mean = diff_Kx.T.mm(self.invK).mm(self.Y_m)
        diff_cov  = -2 * (KXx.T.mm(self.invK).mm(diff_Kx)).T
        

        diff_penalty = torch.zeros(3)[:, None]

        if x[0][2] < self.z_limit:
            print "-----penalty------"
            diff_penalty += torch.Tensor([0, 0, -2 * self.c * (x[0][2] - self.z_limit)])[:, None]
        
        # if (-0.25 > x[0][1] - x[0][0]):
        #     diff_penalty += torch.Tensor([-self.c * 0.01, self.c * 0.01, 0])[:, None]
        
        # if (x[0][1] - x[0][0] > 0.2):
        #     diff_penalty += torch.Tensor([self.c * 0.01, -self.c * 0.01, 0])[:, None]
            

        ############################# arm ###########################################
        # if (-0.27 > x[0][1] - x[0][0]):
        #     diff_penalty += torch.Tensor([-self.c * 0.01, self.c * 0.01, 0])[:, None]
        
        # if (x[0][1] - x[0][0] > 0.17):
        #     diff_penalty += torch.Tensor([self.c * 0.01, -self.c * 0.01, 0])[:, None]
        ############################# arm ###########################################

        ############################# body ###########################################
        if (x[0][1] - x[0][0] > -0.05):
            diff_penalty += torch.Tensor([self.c * 0.01, -self.c * 0.01, 0])[:, None]

        if (-0.265 > x[0][1] - x[0][0]):
            diff_penalty += torch.Tensor([-self.c * 0.01, self.c * 0.01, 0])[:, None]
        ############################# body ###########################################


        diff_object = diff_cov + diff_penalty

        normal     = diff_mean / torch.norm(diff_mean, dim=0)

        projection = torch.eye(self.X.size()[1]) - normal.mm(normal.T)
        S          = torch.mm(projection, diff_object)
        direction  = S / torch.norm(S)

        return normal, direction




                # try:
        #     invK = torch.inverse(K)
        # except:
        #     print 'hogehoge'
        #     X = self.X
        #     sigma = torch.exp(self.sigma_y)
        #     import ipdb; ipdb.set_trace()
        # invL = torch.solve(torch.eye(L.size()[0]), L)[0]
        # invK = torch.mm(invL.T, invL)
from random import Random

#historgram is a list of tuples
#Each tuple corresponds to a distance.
#the first element is the real-world distance, the second element is a list of histogram values.
#Each element in the list of historgram values is a pair - the first
#element of each pair is the distance to report and the second is the
#frequency of that value.  Frequencies are not normalized - they don't
#add up to any constant value.


front_histogram = [(10, [(8,2),(9,5),(10,50),(11,10),(12,2)]),\
                   (15, [(12,1),(13,5),(14,20),(15,50),(16,30),(17,10),(18,2)])]

front_histogram = [(4, [(14, 1),(15, 98),(17, 1)]),   (5, [(10, 99),(11, 1)]),   (6, [(8, 100)]),   (7, [(7, 1),(8, 99)]),   (8, [(8, 100)]),   (9, [(8, 1),(9, 99)]),   (10, [(10, 100)]),   (12, [(12, 96),(13, 4)]),   (14, [(13, 1),(14, 30),(15, 69)]),   (16, [(16, 3),(17, 96),(18, 1)]),   (18, [(19, 97),(20, 3)]),   (20, [(21, 79),(22, 21)]),   (22, [(21, 1),(22, 2),(23, 61),(24, 34),(25, 2)]),   (24, [(24, 1),(25, 32),(26, 64),(27, 3)]),   (26, [(26, 2),(27, 40),(28, 54),(29, 3),(32, 1)]),   (28, [(28, 2),(29, 33),(30, 55),(31, 9),(34, 1)]),   (30, [(30, 2),(31, 33),(32, 53),(33, 11),(34, 1)]),   (32, [(32, 4),(33, 25),(34, 42),(35, 27),(36, 2)]),   (35, [(35, 12),(36, 28),(37, 41),(38, 17),(39, 2)]),   (40, [(33, 1),(38, 2),(39, 6),(40, 34),(41, 34),(42, 17),(43, 5),(44, 1)]),   (45, [(41, 1),(42, 4),(43, 6),(44, 18),(45, 19),(46, 9),(47, 37),(48, 1),(49, 3),(50, 1),(51, 1)]),   (50, [(39, 1),(45, 4),(46, 4),(47, 23),(48, 28),(49, 21),(50, 15),(51, 1),(52, 3)]),   (55, [(44, 1),(45, 5),(46, 5),(47, 19),(48, 22),(49, 28),(50, 15),(51, 2),(52, 2),(53, 1)]),   (60, [(44, 2),(45, 5),(46, 7),(47, 12),(48, 31),(49, 22),(50, 16),(51, 3),(52, 2)]),   (65, [(45, 3),(46, 4),(47, 22),(48, 27),(49, 27),(50, 14),(51, 1),(52, 1),(53, 1)]),   (70, [(45, 2),(46, 4),(47, 18),(48, 18),(49, 21),(50, 27),(51, 5),(52, 5)]),   (75, [(45, 1),(46, 1),(47, 11),(48, 18),(49, 21),(50, 29),(51, 13),(52, 6)]),   (80, [(44, 2),(45, 2),(46, 6),(47, 13),(48, 17),(49, 22),(50, 27),(51, 9),(52, 1),(53, 1)]),   (90, [(43, 2),(44, 5),(45, 13),(46, 19),(47, 29),(48, 13),(49, 14),(50, 3),(53, 2)]),   (100, [(43, 3),(44, 10),(45, 28),(46, 9),(47, 30),(48, 10),(49, 7),(50, 1),(52, 1),(83, 1)]),   (110, [(43, 4),(44, 4),(45, 15),(46, 14),(47, 32),(48, 15),(49, 12),(50, 3),(56, 1)]),   (120, [(42, 0), (43, 2), (44, 2), (45, 8), (46, 4), (47, 12), (48, 26), (49, 21), (50, 15), (51, 4), (52, 5), (53, 1), (54, 0)]),   (140, [(40, 0), (41, 1), (42, 0), (43, 1), (44, 1), (45, 4), (46, 2), (47, 14), (48, 19), (49, 15), (50, 20), (51, 5), (52, 7), (53, 2), (54, 6), (55, 1), (56, 1), (57, 0), (71, 0), (72, 1), (73, 0)]),   (160, [(41, 0), (42, 1), (43, 1), (44, 1), (45, 5), (46, 6), (47, 17), (48, 22), (49, 9), (50, 9), (51, 3), (52, 8), (53, 5), (54, 3), (55, 2), (56, 3), (57, 0), (58, 2), (59, 0), (60, 2), (61, 0), (67, 0), (68, 1), (69, 0)]),   (180, [(39, 0), (40, 1), (41, 3), (42, 4), (43, 6), (44, 11), (45, 16), (46, 1), (47, 10), (48, 15), (49, 6), (50, 4), (51, 3), (52, 5), (53, 2), (54, 4), (55, 0), (56, 1), (57, 2), (58, 3), (59, 0), (60, 1), (61, 0), (62, 2), (63, 0)]),   (200, [(39, 0), (40, 1), (41, 0), (43, 0), (44, 1), (45, 4), (46, 3), (47, 6), (48, 9), (49, 6), (50, 6), (51, 5), (52, 4), (53, 6), (54, 9), (55, 4), (56, 5), (57, 2), (58, 5), (59, 0), (60, 1), (61, 4), (62, 0), (63, 4), (64, 0), (65, 2), (66, 0), (67, 1), (68, 2), (69, 1), (70, 0), (71, 2), (72, 2), (73, 0), (74, 1), (75, 1), (76, 1), (77, 0), (79, 0), (80, 2), (81, 0)]),   (240, [(19, 0), (20, 1), (21, 0), (44, 0), (45, 1), (46, 0), (47, 1), (48, 4), (49, 6), (50, 11), (51, 2), (52, 10), (53, 4), (54, 12), (55, 3), (56, 13), (57, 3), (58, 4), (59, 1), (60, 2), (61, 2), (62, 1), (63, 4), (64, 2), (65, 5), (66, 0), (67, 0), (68, 1), (69, 1), (70, 0), (71, 0), (72, 2), (73, 0), (74, 1), (75, 1), (76, 1), (77, 0), (78, 1), (79, 0)])]


side_histogram = [(2, [(3, 0), (4, 100), (5, 0)]),\
(3, [(2, 0), (3, 100), (4, 0)]),\
(4, [(3, 0), (4, 100), (5, 0)]),\
(5, [(4, 0), (5, 100), (6, 0)]),\
(7, [(6, 0), (7, 100), (8, 0)]),\
(10, [(9, 0), (10, 100), (11, 0)]),\
(12, [(11, 0), (12, 93), (13, 7), (14, 0)]),\
(14, [(13, 0), (14, 71), (15, 29), (16, 0)]),\
(16, [(15, 0), (16, 57), (17, 41), (18, 2), (19, 0)]),\
(18, [(15, 0), (16, 1), (17, 1), (18, 58), (19, 37), (20, 3), (21, 0)]),\
(20, [(18, 0), (19, 5), (20, 40), (21, 54), (22, 1), (23, 0)]),\
(24, [(22, 0), (23, 1), (24, 8), (25, 35), (26, 46), (27, 8), (28, 2), (29, 0)]),\
(28, [(27, 0), (28, 2), (29, 5), (30, 22), (31, 52), (32, 13), (33, 0), (39, 0), (40, 6), (41, 0)]),\
(32, [(31, 0), (32, 1), (33, 0), (39, 0), (40, 99), (41, 0)]),\
(32, [(31, 0), (32, 1), (33, 0), (39, 0), (40, 99), (41, 0)]),\
(36, [(39, 0), (40, 100), (41, 0)]),\
(40, [(39, 0), (40, 100), (41, 0)]),\
(50, [(39, 0), (40, 100), (41, 0)]),\
(60, [(39, 0), (40, 100), (41, 0)]),\
(70, [(39, 0), (40, 100), (41, 0)]),\
(80, [(39, 0), (40, 100), (41, 0)]),\
(100, [(39, 0), (40, 100), (41, 0)]),\
(120, [(39, 0), (40, 100), (41, 0)]),\
(160, [(39, 0), (40, 100), (41, 0)])]


class NoiseModel():
    def __init__(self, histogram):
        self.rand = Random()
        self.histogram = []
        self.normalize_histogram(histogram)

    def normalize_histogram(self, h):
        for dist,hist in h:
            #calculate the total of all the frequencies for this distance
            totalfreq = 0.0
            for noisedist,freq in hist:
                totalfreq = totalfreq + freq
            normhist = []
            normtotal = 0.0
            for noisedist,freq in hist:
                normhist.append((noisedist, normtotal))
                normval = freq/totalfreq
                normtotal = normtotal + normval
            self.histogram.append((dist,normhist))
        #print self.histogram

    def add_noise(self, realdist):
        prevdist, prevhist = self.histogram[0]
        dist1 = -1
        for dist,hist in self.histogram:
            if dist == realdist:
                #only need to use one histogram
                #print "one hist"
                hist1 = hist
                hist2 = hist
                dist1 = dist
                dist2 = dist
                break
            elif dist > realdist:
                #we don't have a histogram for this distance, so we need to interpolate
                #print "two hists"
                hist1 = prevhist
                hist2 = hist
                dist1 = prevdist
                dist2 = dist
                break
            else:
                #not got there yet
                prevdist = dist
                prevhist = hist
        if dist1 == -1:
            #None of the histogram distances were as great as
            #realdist, or all the histogram distances were nore than
            #realdist.  We just have to go with the one with the
            #greatest distance and use that ought to be a better way
            #to do this.
            hist1 = prevhist
            hist2 = prevhist
            dist1 = realdist
            dist2 = realdist

        #OK, now we've got two histograms to interpolate between (they
        #may be the same histogram, but no need to special-case that)
        #print dist1, hist1
        #print dist2, hist2
        
        rnd =self.rand.uniform(0.0, 1.0);
        #print rnd
        prevdist,dummy = hist1[0]
        noisedist1 = -1
        for noisedist,cumfreq in hist1:
            if cumfreq > rnd:
                noisedist1 = prevdist
                break
            prevdist = noisedist
        if noisedist1 == -1:
            noisedist1 = noisedist

        #we only had one historgram because distance was an exact match, so can return now.
        if dist1 == dist2:
            #print "final: " + str(int(noisedist1))
            return noisedist1

        noisedist2 = -1
        prevdist,dummy = hist2[0]
        for noisedist,cumfreq in hist2:
            if cumfreq > rnd:
                noisedist2 = prevdist
                break
            prevdist = noisedist
        if noisedist2 == -1:
            noisedist2 = noisedist

        #print noisedist1, noisedist2
        ratio = float((realdist - dist1)) / (dist2 - dist1)
        #print ratio
        finaldist = ratio * (noisedist2 - noisedist1) + noisedist1
        #print "final: " + str(int(finaldist))
        return int(finaldist)

class FrontNoiseModel(NoiseModel):
    def __init__(self):
        NoiseModel.__init__(self, front_histogram)

class SideNoiseModel(NoiseModel):
    def __init__(self):
        NoiseModel.__init__(self, side_histogram)


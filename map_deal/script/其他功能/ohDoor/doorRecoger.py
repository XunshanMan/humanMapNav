import cv2


class doorRecoger(object):
    def __init__(self):
        print 'create a new recoger.'

        self.threshold = 2000

    def analyzeLine(self, gray, line):
        var = gray[line].var()

        xtotal = gray.shape[1]
        ytotal = gray.shape[0]
        for i in range(xtotal):
            value = gray[line][i]
            gray[int(line - value/255.0 * 250-1)][i] = 255
        cv2.line(gray, (0, line), (xtotal, line), 255, 1)

        cv2.putText(gray, int(var).__str__(), (10, line), 0, 2, (0, 255, 255), 2)

        return var

    def process(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        var1 = self.analyzeLine(gray, 200)
        var2 = self.analyzeLine(gray, 400)
        var3 = self.analyzeLine(gray, 600)

        print('the var in line (200,400,600):', int(var1).__str__(), \
              int(var2).__str__(), int(var3).__str__())

        havedoor = False
        flag = int(var1 > self.threshold) + int(var2 > self.threshold) + int(var3 > self.threshold)
        if flag > 1:
            havedoor = True
            cv2.putText(gray, 'Door', (10, gray.shape[0]-50), 0, 2, (0, 0, 255), 2)

        return gray, havedoor


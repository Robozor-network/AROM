
class mount():
    def __init__(self, parent = None, name = "mount"):
        self.parent = parent
        self.name = name
        print "init mount"

    def connect(self):
        print "connecting"

    def park(self):
        print "park"
        
    def unpark(self):
        print "UNpark"
        
    def setpark(self):
        print "SENpark"
        
    def getpark(self):
        print "GENpark"

    def start(self):
        print "EQmod driver started"
        
    def slew(self):
        raise NotImplementedError()

    def track(self):
        raise NotImplementedError()

    def setPosition(self):
        raise NotImplementedError()

    def getPosition(self):
        raise NotImplementedError()

    def getStatus(self):
        raise NotImplementedError()

    def setAligmentPoint(self):
        raise NotImplementedError()

    def getAligmentPoint(self):
        raise NotImplementedError()

    def setLimits(self):
        raise NotImplementedError

    def getLimits(self):
        raise NotImplementedError()

    def getDriverVer(self):
        print "version is blablabla"
        

class EQmod(mount):
    def __init__(self, parent=None):
        print "init V EQmod"

    def start(self):
        print "EQmod driver started"
        
class SynScan(mount):
    def function():
        pass
        
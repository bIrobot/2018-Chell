class Navx():
    BANGBANG_TOLERANCE = 5.0

    def __init__(self, navx):
        # Communicate w/navX MXP via the MXP SPI Bus.
        self.navx = navx

    def reset(self):
        # Reset navX position data to zero
        self.navx.reset()

    def drive(self, slowSpeed, fastSpeed, setAngle):
        gyroAngle = self.navx.getAngle()
        rotation = self.bangBang(slowSpeed, fastSpeed, setAngle, gyroAngle)
        return rotation

    def bangBang(self, slowSpeed, fastSpeed, setAngle, gyroAngle):
        if gyroAngle == setAngle:
            rotation = 0
        if (gyroAngle - setAngle) < self.BANGBANG_TOLERANCE and (setAngle - gyroAngle) < self.BANGBANG_TOLERANCE:
            if gyroAngle > setAngle:
                rotation = slowSpeed * -1
            else:
                rotation = slowSpeed
        else:
            if gyroAngle > setAngle:
                rotation = fastSpeed * -1
            else:
                rotation = fastSpeed
        return rotation

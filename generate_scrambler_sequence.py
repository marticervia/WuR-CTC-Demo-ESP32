'''
'' Based on Filippo Valmori implementation 
[1] https://drive.google.com/file/d/1yCIwCLw98tVbgFxsbiwO_ACMosbPGNXH/view
'''


'''
'' Function for implementing an additive scrambler (aka synchronous or
'' linear-feedback shift register scrambler) and descrambler.
'''
def AddScramb( InBytes, ConVect, InitState ) :
    InBits = Byte2BitConv(InBytes)
    BitLen = len(InBits)
    RegState = InitState[:]
    OutBits = [0]*BitLen
    for j in range(BitLen) :
        RegBit = (sum([v1*v2 for v1,v2 in zip(RegState,ConVect)])%2)
        OutBits[j] = InBits[j]^RegBit
        RegState[1:] = RegState[:-1]
        RegState[0] = RegBit
    return Bit2ByteConv(OutBits)


'''
'' Function for converting a byte stream into the corresponding bit stream.
'''
def Byte2BitConv( InBytes ) :
    InLen = len(InBytes)
    OutLen = (InLen<<3)
    OutBits = [0]*OutLen
    for j in range(OutLen) :
        ByteIdx = (j>>3)
        BitIdx = 7-(j%8)
        if (InBytes[ByteIdx] >>BitIdx)%2 :
            OutBits[j] = 1
    return OutBits

'''
'' Function for converting a bit stream into the corresponding byte stream.
'''
def Bit2ByteConv( InBits ) :
    InLen = len(InBits)
    OutLen = (InLen>>3)
    OutBytes = [0]*OutLen
    for j in range(InLen) :
        if InBits[j] :
            ByteIdx = (j>>3)
            BitIdx = 7-(j%8)
            OutBytes[ByteIdx] += (1<<BitIdx)
    return OutBytes


if __name__ == "__main__":
    ConVectA = (1,0,0,1,0,0,0)  # Connection vector of additive LFSRs, i.e. z^7+z^4+1 WLAN standard.
    InitStA = [0,0,0,1,1,1,1];	# Initial state of additive LFSRs 0x0F

    """
    Obtain scrambled values for values to generate a LUT for the scrambler
    """

    zeros = range(256)
    scrambled_vals = AddScramb(zeros, ConVectA, InitStA)

    print("static const uint8_t scrambler_table[]  = [")
    for i in range(255):
        print(f"{scrambled_vals[i]},", end = '')
        if i % 16 == 0:
            print("")
    print(f"{scrambled_vals[255]}]")





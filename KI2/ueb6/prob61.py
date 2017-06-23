def stateTrans(x_2, x_1):
    if(x_1 == 'H' and x_2 == 'H'):
        return 0.85
    elif(x_1 == 'B' and x_2 == 'B'):
        return 0.7
    elif(x_1 == 'H' and x_2 == 'B'):
        return 0.15
    elif(x_1 == 'B' and x_2 == 'H'):
        return 0.3

def eviEmi(x, e):
    if(x == 'H' and e == 'DM'):
        return 0.4
    elif(x == 'B' and e == 'DM'):
        return 0.6
    elif(x == 'H' and e == 'J'):
        return 0.7
    elif(x == 'B' and e == 'J'):
        return 0.3

prob = 0.0
x0 = 'H';
for x1 in ['H', 'B']:
    for x2 in ['H', 'B']:
        for x3 in ['H', 'B']:
            probTmp = stateTrans(x1, x0) * eviEmi(x1, 'DM') * stateTrans(x2, x1) * eviEmi(x2, 'DM') * stateTrans(x3, x2) * eviEmi(x3, 'DM') 
            print(str(x0) + " " + str(x1) + " " + str(x2) + " " + str(x3) + " " + str(probTmp))
            prob += probTmp

print(prob)

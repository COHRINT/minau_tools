import csv
import matplotlib.pyplot as plt
import os



class DataGraph:
    def __init__(self):
        NUM_ASSETS = 3
        self.file = [i for i in os.listdir('.') if i[-3:]=='csv'][0]
        self.reader = csv.DictReader(
            open(self.file)
        )

        self.data = []
        for row in self.reader:
            self.data.append(row)



        if NUM_ASSETS == 2:
            self.plot('Ownship_Error3')
            self.plot('Blue_Error3')
            self.plot('Red_Error3')
            self.plot('Ownship_Error4')
            self.plot('Blue_Error4')
            self.plot('Red_Error4')
        elif NUM_ASSETS == 3:
            self.plot('Ownship_Error3')
            self.plot('Blue_Error34')
            self.plot('Blue_Error35')
            self.plot('Red_Error3')

            self.plot('Ownship_Error4')
            self.plot('Blue_Error43')
            self.plot('Blue_Error45')
            self.plot('Red_Error4')

            self.plot('Ownship_Error5')
            self.plot('Blue_Error54')
            self.plot('Blue_Error53')
            self.plot('Red_Error5')
        else:
            print('We are not set up for ' + str(NUM_ASSETS) + ' assets yet')

        

    def plot(self,data_name):
        avg_err = [float(self.data[i]['Avg_' + data_name]) for i in range(len(self.data))]
        std_err = [2*float(self.data[i]['Std_' + data_name]) for i in range(len(self.data))]
        x = range(len(self.data))
        plt.figure()
        (_, caps, _) = plt.errorbar(x,avg_err, yerr=std_err, capsize=4, elinewidth=2, ecolor='green', color='b')

        for cap in caps:
            cap.set_color('green')
            cap.set_markeredgewidth(2)
        plt.xlabel('Test Number')
        plt.ylabel('Error')
        plt.title('Repeat Tester Estimation Error: ' + data_name)
        plt.grid()
        plt.savefig(data_name+".png")
        plt.close()



if __name__ == "__main__":
    d = DataGraph()



exit()

file = [i for i in os.listdir('.') if i[-3:]=='csv'][0]
print(file)
print(file[:-4])

reader = csv.DictReader(
            open(file)
        )

data = []
for row in reader:
    data.append(row)

asset3_ownship = [float(data[i]['Avg_Ownship_Error3']) for i in range(len(data))]
asset3_2std = [2*float(data[i]['Std_Ownship_Error3']) for i in range(len(data))]
asset3_blue4 = [float(data[i]['Avg_Blue_Error3']) for i in range(len(data))]
asset3_red = [float(data[i]['Avg_Red_Error3']) for i in range(len(data))]

asset4_ownship = [float(data[i]['Avg_Ownship_Error4']) for i in range(len(data))]
asset4_blue3 = [float(data[i]['Avg_Blue_Error4']) for i in range(len(data))]
asset4_red = [float(data[i]['Avg_Red_Error4']) for i in range(len(data))]

x = range(len(data))

print(asset4_ownship)
# plt.plot(x,asset3_ownship)
# plt.plot(x,asset4_ownship)
# plt.plot(x,asset3_blue4)
# plt.plot(x,asset4_blue3)
plt.figure()


(_, caps, _) = plt.errorbar(x,asset3_ownship, yerr=asset3_2std, capsize=4, elinewidth=2, ecolor='green', color='b')

for cap in caps:
    cap.set_color('green')
    cap.set_markeredgewidth(2)
# plt.plot(x,asset3_ownship, color='b')


# plt.errorbar(x,asset3_ownship, yerr=asset3_2std)

# plt.legend(['Dory Red Estimates','Guppy Red Estimates'])
plt.xlabel('Test Number')
plt.ylabel('Error')
plt.title('Repeat Tester Estimation Error')
plt.grid()
plt.savefig(file[:-4]+"_own.png")
plt.show()


print(data[0]['Time_to_Spot'])
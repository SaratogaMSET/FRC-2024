import pandas

df = pandas.read_csv("8.26.pm.18.2.24.csv", index_col="Timestamp")
df = df.filter(regex = 'NT:/AdvantageKit/Vision/Camera \d/Pipeline Result/targets/\d/.+')

df = df.filter(regex = 'NT:/AdvantageKit/Vision/Camera \d/Pipeline Result/targets/\d/best_camera_to_target/rotation|translation/.+')

print(df) #TODO: Speak With the people about this and probably do args parse for io files.
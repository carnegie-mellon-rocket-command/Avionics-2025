import pandas as pd

dl = pd.read_csv('VDF.csv')

print(list(dl[" altitude_raw (ft)"]))
print(list(dl[" gAccelFiltered (ft/s^2)"]))
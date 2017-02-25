import requests
import json

r = requests.get('http://localhost:8000/#')
r.json()

print r

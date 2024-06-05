import sqlite3


class DatabaseConnector():
    def __init__(self, path):
        self.path = path
    
    def insert_in_table(self,body):
        conn = sqlite3.connect(self.path)
        cursor = conn.cursor()
        cursor.execute('INSERT INTO tabelax VALUES ?','body')
    

    

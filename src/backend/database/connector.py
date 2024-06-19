import sqlite3
from sqlite3 import Error
from typing import Any, List, Dict, Tuple
from schemas import *


def create_connection(db_path:str) -> Any:
    """
    Cria conexão com o database

    Parameters
    ----------
    db_path : str
        caminho do arquivo do database
    
    Returns
    -------
    sqlite3.conn
        conexão com o banco de dados

    Exemples
    --------
    >>> create_connection(db_path="./database.db") # database na pasta raiz
    <sqlite3.Connection object at 0x0000021D56AE06D0>
    """
    conn = None
    try:
        conn = sqlite3.connect(db_path) 
    except Error as e:
        print(e)

    print(conn)

    return conn


def handle_object_data(obj: Any) -> Tuple[Any]:
    """
    Função que trata dados do tipo object e dict

    Parameter
    ---------
    obj: dict | BaseModel
        Objeto que contém dados do query
    
    Returns
    -------
    Tuple[Tuple["str"], Tuple[Any]]
        Tupla que contem valores das chaves do objeto com os valores dos objetos

    Exemples
    --------
    >>> handle_object_data(obj = {
        "user_name" : "Bananildo",
        "user_email": "Bananildo@gmail.com",
        "user_password": "123",
        "user_role": 3}
    )
    ("user_name","user_email","user_password", "user_role"), ("Bananildo", "Bananildo@gmail.com", "123", 3)

    """

    if type(obj) is dict:
        print("Objeto dicionário")
        objects_keys = tuple(obj.keys())
        objects_values = tuple(obj.values())
    else:
        print("Objeto schema")
        objects_keys = tuple(obj.__dict__.keys())
        objects_values = tuple(obj.__dict__.values())

    return objects_keys, objects_values


def execute_query(conn: Any, query: str, params=Tuple) -> None:
    """
    Função que executa o query e comita as alterações

    Parameters
    ----------
    conn   : sqlite3.connect
        conexão com o banco de dados
    query  : str
        string de execução do query
    params : Tuple 
        parâmetros de execução do query

    Returns
    -------
    None

    Exemples
    --------
    >>> execute_query(
        conn=sqlite3.connect("./database.db"),
        query="INSERT INTO Users ('user_name', 'user_email', 'user_password', 'user_role') VALUES (?, ?, ?, ?)",
        params = ("user", "user@gmail.com", "123_user", 3)
    )
    """
    try:
        cursor = conn.cursor()
        cursor.execute(query, params)
        conn.commit()
    except Error as e:
        print(e)


class DatabaseConnection:
    def __init__(self, db_path:str) -> None :
        """
        Classe que conecta com o banco de dados

        Parameters
        ----------
        db_path : str
            Caminho do arquivo referente ao banco de dados
        """
        self.connection = create_connection(db_path=db_path)

    def query_database(self, operation: str, table_name: str, **kwargs) -> Any:
        """
        Função comum que executa as operações de INSERT, SEARCH, UPDATE e DELETE no database
                
        Parameters
        ----------
        operation : str
            Tipo de operação a ser executada no database
        table_name : str
            Nome da tabela que sofrerá a ação
            
        Returns
        -------
            Any:
                Pode retornar o resultado de um SELECT (List[Dict[str, Any]]), como mensagens de sucesso ou erro das demais operações

        Exemples
        --------
        Utilizações dos métodos query:
        1.  SELECT -> O método select deve ser utilizado passando os parâmetros `columns` e `where`, no qual o último é opcional
        >>> query_database(
            operation="SELECT", 
            table_name="Users", 
            columns="*", 
            where={"user_id" = 1}
            )
            [{'user_id': 1, 'user_email': 'user@gmail.com', 'user_password': '123', 'user_role': 3, 'user_name': 'user'}]

        2. INSERT -> O método de insert possui três formas de ser usado, tanto com parâmetros como com dados em forma de objetos ou dicionário.
        É importante ressaltar que quando forem utilizados objetos e dicionários como dados de entrada, é preciso passá-los no parâmetro `data`
        >>> #=== Com parâmetros ===#
            query_database(
            operation="INSERT", 
            table_name="Users", 
            user_name="user", 
            user_email="user@gmail.com", 
            user_password="123"
            )

        >>> #=== Com dicionário ===#
            query_database(
            operation="INSERT", 
            table_name="Users",
            data={
                "user_name": "user", 
                "user_email": "user@gmail.com", 
                "user_password": "123",
                "user_role" : 1}
            )

        >>> #=== Com objeto ===#
            class User(BaseModel):
                user_email: str
                user_password: str
                user_role: int
                user_name: str
            user_test = User(
                user_email="user@gmail.com",
                user_name="user",
                user_password="123",
                user_role=1
            )
            query_database(operation="INSERT", table_name="Users", data=user_test)

        3. UPDATE -> O método de update possui duas formas de ser usado, tanto com dados em forma de objetos ou dicionário.
        É importante ressaltar que quando forem utilizados objetos e dicionários como dados de entrada, é preciso passá-los no parâmetro `data`
        >>> #=== Com dicionário ===#
            query_database(
            operation="UPDATE", 
            table_name="Users", 
            data={
                "user_name": "user_updated", 
                "user_email": "user_updated@gmail.com", 
                "user_password": "123456", 
                "user_role" : 12}, 
                where={"user_id" : 1}
            )

        >>> #=== Com objeto ===#
            class User(BaseModel):
                user_email: str
                user_password: str
                user_role: int
                user_name: str
            user_test = User(
                user_email="user_updated@gmail.com",
                user_name="user_updated",
                user_password="123456",
                user_role=5
            )
            query_database(
            operation="INSERT", 
            table_name="Users", 
            data=user_test,
            where={"user_id" = 1}
        
        4. DELETE -> O método delete recebe apenas o parâmetro opcional `where`, como filtro do comando 
        >>> query_database(
        operation="DELETE",
        table_name="Users",
        where={"user_id": 1}
        )
        """
        
        operation = operation.upper()
        result: List[Dict[str, Any]] = []

        try:
            with self.connection:
                cursor = self.connection.cursor()
                match operation: 
                    case "SELECT":
                            columns = kwargs.get('columns', '*')
                            query = f"{operation} {columns} FROM {table_name}"

                            params = ()
                            if "where" in kwargs:
                                where_clause = ' AND '.join([f"{k} = ?" for k in kwargs['where'].keys()]) 
                                query += f" WHERE {where_clause}"
                                params = tuple(kwargs['where'].values()) 
                            
                            print(query)
        
                            cursor.execute(query, params) 
                            rows = cursor.fetchall()
                            columns_names: List[str] = [description[0] for description in cursor.description] 
                            result = [dict(zip(columns_names, row)) for row in rows]
                    
                    case "INSERT":
                        if "data" in kwargs:
                            columns, values = handle_object_data(kwargs["data"])
                            placeholders = ", ".join("?" * len(columns)) 
                            query = f"{operation} INTO {table_name} {columns} VALUES ({placeholders})"
                        else:
                            columns = ", ".join(kwargs.keys())
                            values = tuple(kwargs.values()) 
                            placeholders = ", ".join("?" * len(kwargs))
                            query = f"{operation} INTO {table_name} ({columns}) VALUES ({placeholders})"
                        
                        print(query)
                        execute_query(conn=self.connection, query=query, params=values) 

                        return "Dados inserido com sucesso"
                    
                    case "UPDATE":
                            if "data" in kwargs:
                                columns, values = handle_object_data(kwargs["data"])
                                set_clause = ", ".join([f"{k} = ?" for k in columns])
                                params = values
                                print(set_clause, params)
                            else:
                                set_clause = ', '.join([f"{k} = ?" for k in kwargs['data'].keys()])
                                params = tuple(kwargs['columns'].values()) 
                            
                            query = f"{operation} {table_name} SET {set_clause}" 
                            if 'where' in kwargs:
                                where_clause = ' AND '.join([f"{k} = ?" for k in kwargs['where'].keys()]) 
                                query += f"WHERE {where_clause}"
                                params += tuple(kwargs['where'].values())

                            execute_query(conn=self.connection, query=query, params=params)

                            print(query)

                            return "Dados atualizado com sucesso"
                    
                    case "DELETE":
                            query = f"{operation} FROM {table_name}"

                            if "where" not in kwargs:
                                params = ()

                            where_clause = ' AND '.join([f"{k} = ?" for k in kwargs['where'].keys()])
                            query += f" WHERE {where_clause}"
                            params = tuple(kwargs['where'].values())

                            print(query)

                            execute_query(conn=self.connection, query=query, params=params)

                            return "Dados deletado com sucesso"

                    case _:
                        print(f"Operação {operation} não suportada.")
                        return f"Operação {operation} não suportada."
                
        except Error as e:
            print(e)

        return result
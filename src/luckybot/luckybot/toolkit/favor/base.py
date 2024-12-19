from abc import ABC, abstractmethod
from typing import List

class Favor(ABC):
    @abstractmethod
    def create_group(self,group_name) -> str:
        pass
    @abstractmethod
    def get_groups(self) -> List[dict]:
        pass
    @abstractmethod
    def rename_group(self,group_id, group_name):
        pass
    @abstractmethod
    def del_group(self,group_name=None, group_id=None):
        pass
    @abstractmethod
    def get_group_id(self,group_name) -> str:
        pass
    @abstractmethod
    def get_symbols(self,group_name=None) -> List[str]:
        pass
    @abstractmethod
    def del_from_group(self,code, entity_type="stock", group_name=None, group_id=None):
        pass
    @abstractmethod
    def add_symbols_to_group(self,codes, entity_type="stock", group_name=None, group_id=None):
        pass
    @abstractmethod
    def del_symbols_from_group(self,codes, entity_type="stock", group_name=None, group_id=None):
        pass
    @abstractmethod
    def del_all_from_group(self,entity_type="stock", group_name=None, group_id=None):
        pass
    @abstractmethod
    def add_to_group(self,code, entity_type="stock", group_name=None, group_id=None):
        pass
    @abstractmethod
    def update_favor(self,symbols:list[str], group_name:str):
        pass        
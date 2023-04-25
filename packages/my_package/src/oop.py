#!/usr/bin/env python3
#tehtud
class Trainers():
    def __init__(self, stamina: int, color: str):
        self.stamina = stamina
        self.color = color

    def __str__(self) -> str:
        return f'Trainers: (\'{self.stamina}\', {self.color})'
    
    def __repr__(self) -> str:
        return self.__str__()  

class Member(): 
    def __init__(self, name: str, age: int, trainers: Trainers):
        self.name = name
        self.age = age
        self.trainers = trainers

    def get_all_gyms(self) -> list:
        pass
        #liige saali
    def get_gyms(self) -> list:
        pass
        #liige saali
        
    def __str__(self) -> str:
        return f'Member: ({self.name}, {self.age}, {self.trainers})'

    def __repr__(self) -> str:
        return self.__str__() 
        
class Gym():
    def __init__(self, name: str, max_members_number: int):
        self.name = name
        self.max_members_number = max_members_number
        self.__members = []

    def add_member(self, member: Member) -> Member:
        if isinstance(member, Member) and member not in self.__members:
            return member
    
    def can_add_member(self, member: Member) -> bool:
        member = self.add_member()
        if member not in self.__members:
            return True
        else:
            return False
        #Meetod tagastab ``True``, kui liiget saab saali lisada, muul juhul tagastab ``False``. Mis tähendab, kui saab lisada, mis parameeter selleks on???
    def remove_member(self, member: Member):
        pass

    def get_total_stamina(self) -> int:
        pass

    def get_members_number(self) -> int:
        return self.__members

    def get_all_members(self) -> list:
        pass

    def get_average_age(self) -> float:
        pass
        
class City():
    def __init__(self, max_gym_number: int):
        self.max_gym_number = max_gym_number
        self.__gyms = []

    def build_gym(self, gym: Gym) -> Gym:
        if self.can_build_gym() == True: 
            self.__gyms.append(gym)
        return gym

    def can_build_gym(self) -> bool:
        if self.build_gym not in self.__gyms and len(self.__gyms) <= self.max_gym_number:
            return True
        else:
            return False
    
    def destroy_gym(self):
        #self.__gyms.remove()
        pass

    def get_max_members_gym(self) -> list:
        pass

    def get_max_stamina_gyms(self) -> list:
        pass

    def get_max_average_ages(self) -> list:
        pass

    def get_min_average_ages(self) -> list:
        pass

    def get_gyms_by_trainers_color(self, color: str) -> list:
        pass

    def get_gyms_by_name(self, name: str) -> list:
        pass

    def get_all_gyms(self) -> list:
        return self.__gyms

if __name__ == '__main__':

    city1 = City(5)
    city1.build_gym("Leola")
    gym1 = Gym("Leola", 10)
    trainer1 = Trainers(5, "Punane")
    member1 = Member("Romet", 10, trainer1)
        
    print(gym1.add_member(member1))
    #print(city1.can_build_gym())
    #print(city1.get_all_gyms())
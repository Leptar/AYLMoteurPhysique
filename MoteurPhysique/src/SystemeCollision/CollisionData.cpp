#include "CollisionData.h"

#include <functional>

void CollisionData::add(Contact contact)
{
    contacts.push_back(contact);
}

void CollisionData::remove(const Contact& contact)
{
    for (auto it = contacts.begin(); it != contacts.end(); ++it) {
        if (it->equal(contact.c1, contact.c2)) {
            contacts.erase(it);
        }
    }
}

void CollisionData::clear()
{
    contacts.clear();
}

#include "OntologyView.hpp"
#include <owlapi/Vocabulary.hpp>

namespace templ {
namespace gui {

OntologyTreeModel::OntologyTreeModel(const owlapi::model::OWLOntology::Ptr& ontology,
        const owlapi::model::IRI& root,
        QObject* parent)
    : QAbstractItemModel(parent)
{
    owlapi::model::OWLOntologyAsk mAsk(ontology);
    mIris = mAsk.allSubClassesOf(root, false);

    LOG_WARN_S << "FOUND FUNCTIONALITES" << mIris;
}

OntologyTreeModel::~OntologyTreeModel()
{}

QVariant OntologyTreeModel::data(const QModelIndex& index, int role) const
{
    LOG_WARN_S << "QUERY data: " << index.row() << " role: " << role;
    if(!index.isValid())
    {
        return QVariant();
    }
    if(role != Qt::DisplayRole)
    {
        return QVariant();
    }

    owlapi::model::IRI iri = mIris[index.row()];
    LOG_WARN_S << "RETURN DATA: " << iri << "for row: " << index.row();
    return QVariant( iri.toString().c_str() );
}

QModelIndex OntologyTreeModel::index(int row, int column,
                 const QModelIndex& parent) const
{
    LOG_WARN_S << "GET INDEX" << row << "/" << column;

    if(!hasIndex(row, column, parent))
    {
        return QModelIndex();
    }

    if(!parent.isValid())
    {
        return createIndex(row, column);
    }


    return QModelIndex();
}

QModelIndex OntologyTreeModel::parent(const QModelIndex& index) const
{
    LOG_WARN_S << "GET parent: current index.row() " << index.row();
    return QModelIndex();
}
int OntologyTreeModel::rowCount(const QModelIndex& parent) const
{
    if(parent.row() == -1)
    {
        LOG_WARN_S << "GET rowCount" << mIris.size();
        return mIris.size();
    } else {
        LOG_WARN_S << "GET rowCount no iris: parent.row() " << parent.row();
        return 0;
    }
}
int OntologyTreeModel::columnCount(const QModelIndex& parent) const
{
    LOG_WARN_S << "GET COLUMN COUNT: parent.row() " << parent.row();
    return 1;
}


} // end namespace gui
} // end namespace templ

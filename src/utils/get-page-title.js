import defaultSettings from '@/settings'

const title = defaultSettings.title || 'HILMAP'

export default function getPageTitle(pageTitle) {
  if (pageTitle) {
    return `${pageTitle} - ${title}`
  }
  return `${title}`
}
